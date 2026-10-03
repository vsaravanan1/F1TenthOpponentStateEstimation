#!/usr/bin/python3
"""
reference_manager.py  (dynamic MPPI)

Geometry helpers for the dynamic MPPI node. Two roles, kept separate on purpose:

  * CONVERTER side (runs OUTSIDE MPPI): `path_to_waypoints` turns a raw ordered
    (x, y) path into [x, y, psi, v] waypoints MPPI can track verbatim. No raceline
    geometry needed.

  * MPPI side (the ONLY geometry MPPI runs itself): `build_return` generates a
    smooth, curvature-continuous spline from the car's current (off-line) pose
    back onto the centerline, and `splice` assembles the full active loop
    (centerline -> divert -> return -> centerline).

Everything is built on a single periodic cubic-spline model of the centerline, so
projection (xy -> s, d) and reconstruction (s, d -> xy) use the SAME smooth basis.
That is what guarantees the return rejoins the line with matching position,
heading and curvature, with no zig-zag from piecewise normals.

Waypoint columns (match the existing node / Spielberg CSV after load):
    0: x   (map frame, m)
    1: y   (map frame, m)
    2: psi (CSV/TUM convention, psi = pi/2 - heading)
    3: v   (ALREADY-NORMALISED throttle, in [min_throttle, max_throttle])

Yaw note: the node consumes column 2 as ref_yaw = col2 + pi/2. Empirically the true
heading is -psi + pi/2, so the node's yaw reference is off by 2*psi (position cost
dominates, so it drives). We do NOT change that here; we store divert/return yaw the
SAME way the CSV does (psi = pi/2 - heading) so a generated point behaves exactly
like a centerline point. If you ever fix the node's yaw, change `_heading_to_psi`.
"""

import numpy as np

try:
    from scipy.interpolate import CubicSpline, make_interp_spline
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False


def _wrap_pi(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _smootherstep(e0, e1, x):
    t = np.clip((x - e0) / max(e1 - e0, 1e-9), 0.0, 1.0)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def _resample_polyline(xy, spacing, carry=None):
    d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    cl = np.concatenate(([0.0], np.cumsum(d)))
    total = cl[-1]
    n = max(int(np.round(total / spacing)) + 1, 2)
    u = np.linspace(0.0, total, n)
    xy_rs = np.stack([np.interp(u, cl, xy[:, 0]), np.interp(u, cl, xy[:, 1])], axis=1)
    if carry is None:
        return xy_rs, None
    return xy_rs, np.interp(u, cl, carry)


class ReferencePlanner:
    """Periodic cubic-spline centerline + divert/return reference assembly."""

    def __init__(self, raceline, closed=True):
        self.raceline = np.asarray(raceline, dtype=float).copy()
        self.closed = closed
        self.N = len(self.raceline)
        xy = self.raceline[:, :2]

        if closed:
            seg = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
        else:
            seg = np.append(np.linalg.norm(np.diff(xy, axis=0), axis=1), 0.0)
        self.seg = seg
        self.s = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
        self.total_s = float(np.sum(seg))

        s_closed = np.concatenate([self.s, [self.total_s]])
        xy_closed = np.vstack([xy, xy[:1]])
        if _HAVE_SCIPY and closed:
            self._cs = CubicSpline(s_closed, xy_closed, bc_type="periodic", axis=0)
            self._cs1 = self._cs.derivative(1)
            self._cs2 = self._cs.derivative(2)
        else:
            self._cs = None

        dxy = np.roll(xy, -1, axis=0) - xy
        self.heading = np.arctan2(dxy[:, 1], dxy[:, 0])

    # centerline model
    def c(self, s):
        s = np.atleast_1d(np.asarray(s, float)) % self.total_s
        if self._cs is not None:
            return self._cs(s)
        sa = np.append(self.s, self.total_s)
        return np.stack([np.interp(s, sa, np.append(self.raceline[:, 0], self.raceline[0, 0])),
                         np.interp(s, sa, np.append(self.raceline[:, 1], self.raceline[0, 1]))], axis=1)

    def tangent(self, s):
        s = np.atleast_1d(np.asarray(s, float)) % self.total_s
        if self._cs is not None:
            t = self._cs1(s)
        else:
            eps = 0.05
            t = (self.c(s + eps) - self.c(s - eps)) / (2 * eps)
        return t / np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)

    def left_normal(self, s):
        t = self.tangent(s)
        return np.stack([-t[:, 1], t[:, 0]], axis=1)

    def heading_at(self, s):
        t = self.tangent(s)
        return np.arctan2(t[:, 1], t[:, 0])

    # Frenet
    def _nearest_idx(self, pts):
        d2 = np.sum((self.raceline[None, :, :2] - pts[:, None, :]) ** 2, axis=2)
        return np.argmin(d2, axis=1)

    def project(self, pts):
        pts = np.asarray(pts, dtype=float)
        if pts.ndim == 1:
            pts = pts[None, :]
        idx = self._nearest_idx(pts)
        s = self.s[idx].astype(float).copy()
        if self._cs is not None:
            for _ in range(4):
                c = self._cs(s % self.total_s)
                c1 = self._cs1(s % self.total_s)
                c2 = self._cs2(s % self.total_s)
                diff = c - pts
                f1 = np.sum(diff * c1, axis=1)
                f2 = np.sum(c1 * c1, axis=1) + np.sum(diff * c2, axis=1)
                s = s - f1 / np.where(np.abs(f2) < 1e-9, 1e-9, f2)
            s = s % self.total_s
        c = self.c(s)
        ln = self.left_normal(s)
        d = np.sum((pts - c) * ln, axis=1)
        return s, d, idx

    def frenet_to_xy(self, s, d):
        s = np.atleast_1d(np.asarray(s, float))
        d = np.atleast_1d(np.asarray(d, float))
        return self.c(s) + d[:, None] * self.left_normal(s)

    @staticmethod
    def _heading_to_psi(heading):
        return _wrap_pi(np.pi / 2.0 - heading)

    # converter-side
    @staticmethod
    def path_to_waypoints(xy, v_value, spacing=0.1):
        xy = np.asarray(xy, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2:
            raise ValueError("path must be (M>=2, 2)")
        xy_rs, _ = _resample_polyline(xy, spacing)
        d = np.gradient(xy_rs, axis=0)
        psi = _wrap_pi(np.pi / 2.0 - np.arctan2(d[:, 1], d[:, 0]))
        v = np.full(len(xy_rs), float(v_value))
        return np.stack([xy_rs[:, 0], xy_rs[:, 1], psi, v], axis=1)

    @staticmethod
    def _is_forward(xy, max_turn_deg=35.0):
        """True if no consecutive-segment turn exceeds max_turn_deg (no fold/kink)."""
        d = np.diff(xy, axis=0)
        n = np.linalg.norm(d, axis=1, keepdims=True)
        d = d / np.maximum(n, 1e-9)
        dots = np.clip(np.sum(d[:-1] * d[1:], axis=1), -1.0, 1.0)
        return bool(np.all(dots > np.cos(np.radians(max_turn_deg))))

    # MPPI-side: return spline
    def build_return(self, start_xy, start_heading,
                     return_len=5.0, spacing=None, start_v=None, v_scale=1.0,
                     max_return_len=None):
        """
        Simple return: straight line from the car's current position to a point
        ~return_len metres ahead ON the centerline, then it's back on the line.
        Not curvature-continuous, but robust and never swings the wrong way.
        """
        if spacing is None:
            spacing = float(np.median(self.seg))

        start_xy = np.asarray(start_xy, float).reshape(1, 2)
        s0, d0, _ = self.project(start_xy)
        s0 = float(s0[0])

        # target: a point return_len ahead along the centerline, offset 0 (on the line)
        s_target = s0 + return_len
        target_xy = self.frenet_to_xy(np.array([s_target]), np.array([0.0]))[0]

        # straight line from start to target, resampled at `spacing`
        p0 = start_xy[0]
        dist = float(np.hypot(*(target_xy - p0)))
        n = max(int(np.ceil(dist / spacing)) + 1, 2)
        t = np.linspace(0.0, 1.0, n)
        ret_xy = p0[None, :] * (1 - t)[:, None] + target_xy[None, :] * t[:, None]

        # heading from the straight line, stored in CSV psi convention
        d = np.gradient(ret_xy, axis=0)
        ret_psi = self._heading_to_psi(np.arctan2(d[:, 1], d[:, 0]))

        # speed: blend start_v up to the raceline throttle at the target
        v_axis = np.append(self.raceline[:, 3], self.raceline[0, 3])
        s_axis = np.append(self.s, self.total_s)
        if start_v is None:
            start_v = float(np.interp(s0 % self.total_s, s_axis, v_axis))
        v_target = float(np.interp(s_target % self.total_s, s_axis, v_axis))
        w = np.linspace(0.0, 1.0, n)
        ret_v = np.clip((1 - w) * start_v + w * v_target,
                        self.raceline[:, 3].min(), self.raceline[:, 3].max())

        wps = np.stack([ret_xy[:, 0], ret_xy[:, 1], ret_psi, ret_v], axis=1)
        return wps, {"s0": s0, "s_end": s_target % self.total_s, "end_xy": ret_xy[-1].copy()}

    # splice
    def splice(self, divert_wps, return_wps):
        divert_wps = np.asarray(divert_wps, float)
        return_wps = np.asarray(return_wps, float)
        if len(return_wps) > 1 and np.hypot(*(return_wps[0, :2] - divert_wps[-1, :2])) < 1e-3:
            return_wps = return_wps[1:]

        s_start = float(self.project(divert_wps[:1, :2])[0][0])
        s_end = float(self.project(return_wps[-1:, :2])[0][0])
        if s_end <= s_start:
            raise ValueError("divert+return wraps the start/finish line; "
                             "trigger the divert away from the seam")

        before = self.raceline[self.s < s_start]
        after = self.raceline[self.s > s_end]

        divert_wps = divert_wps.copy()
        v_axis = np.append(self.raceline[:, 3], self.raceline[0, 3])
        s_axis = np.append(self.s, self.total_s)
        v_entry = float(np.interp(s_start % self.total_s, s_axis, v_axis))
        k = min(6, len(divert_wps))
        if k > 1:
            w = _smootherstep(0.0, 1.0, np.linspace(0, 1, k))
            divert_wps[:k, 3] = (1 - w) * v_entry + w * divert_wps[:k, 3]

        active = np.vstack([before, divert_wps, return_wps, after])
        info = {
            "n_before": int(len(before)),
            "n_divert": int(len(divert_wps)),
            "n_return": int(len(return_wps)),
            "divert_end_idx": int(len(before) + len(divert_wps) - 1),
            "return_end_idx": int(len(before) + len(divert_wps) + len(return_wps) - 1),
            "return_end_xy": return_wps[-1, :2].copy(),
            "s_start": s_start % self.total_s,
            "s_end": s_end % self.total_s,
        }
        return active, info