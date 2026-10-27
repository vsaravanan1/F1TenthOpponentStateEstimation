#!/usr/bin/env python3
import numpy as np
import pandas as pd
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev


class RacetrackUtilities:
    """Frenet frame for a closed track given by a centerline CSV.

    s is true arc length [m] along a smoothed centerline spline and d is the
    signed perpendicular offset from it (positive = left). The spline is
    parameterised so that u = s / arclength, so splev(s / arclength, ...) on
    centerline_spline or racetrack_widths_spline evaluates at s, and
    convert_to_frenet / convert_to_cartesian are exact inverses. s wraps at
    arclength everywhere.
    """

    # splprep smoothing per point. The CSV is pixel-quantised (~6 cm grid); this keeps
    # the spline within ~7 cm of it while removing the stair-step curvature noise
    CENTERLINE_SMOOTHING = 0.0003
    FRENET_SPACING = 0.01   # [m] spacing of the dense centerline samples used for projection
    REFIT_SPACING = 0.1     # [m] sample spacing for the arc-length re-parameterisation

    def __init__(self, csv_file):
        self.centerline_tree = None
        self.centerline = None
        self.racetrack_widths = None

        centerline_shape = self.parse_csv(csv_file)
        self.num_points = centerline_shape[0]

        self.centerline_spline, self.arclength = self.create_centerline_spline()
        self.build_frenet_lookup()
        # s / arclength of each CSV centerline point in this frame
        self.centerline_s_norm = self.convert_to_frenet_arr(self.centerline)[:, 0] / self.arclength
        self.racetrack_widths_spline = self.create_widths_spline()

    def parse_csv(self, csv_file):
        raceline_df = pd.read_csv(csv_file)
        self.centerline = raceline_df[["x_m", "y_m"]].to_numpy()
        self.racetrack_widths = raceline_df[["w_tr_left_m", "w_tr_right_m"]].to_numpy()
        self.centerline_tree = KDTree(self.centerline)
        return self.centerline.shape

    def create_centerline_spline(self):
        """Smoothed periodic centerline spline with u = s / arclength."""
        # closed loop: repeat the first point so the closing segment is part of the fit
        loop = np.vstack([self.centerline, self.centerline[:1]])
        chord = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(loop, axis=0), axis=1))])
        m = len(self.centerline)
        rough, _ = splprep([loop[:, 0], loop[:, 1]], u=chord / chord[-1],
                           s=m * self.CENTERLINE_SMOOTHING, k=3, per=True)

        # true arc length of the smoothed curve, then refit with u = s / arclength
        u_dense = np.linspace(0.0, 1.0, 20 * m + 1)
        pts = np.column_stack(splev(u_dense, rough))
        s_dense = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        arclength = s_dense[-1]

        s_fit = np.linspace(0.0, arclength, int(arclength / self.REFIT_SPACING) + 1)
        x_fit, y_fit = splev(np.interp(s_fit, s_dense, u_dense), rough)
        tck, _ = splprep([x_fit, y_fit], u=s_fit / arclength, s=0, k=3, per=True)
        return tck, arclength

    def build_frenet_lookup(self):
        """Dense centerline samples + KDTree for convert_to_frenet_arr."""
        u = np.linspace(0.0, 1.0, int(self.arclength / self.FRENET_SPACING), endpoint=False)
        self._lookup_s = u * self.arclength
        self._lookup_xy = np.column_stack(splev(u, self.centerline_spline))
        dx, dy = splev(u, self.centerline_spline, der=1)
        mag = np.hypot(dx, dy)
        self._lookup_tangent = np.column_stack([dx / mag, dy / mag])
        self._lookup_kappa = self.curvature(u)
        self._lookup_tree = KDTree(self._lookup_xy)

    def create_widths_spline(self):
        """Periodic spline of [left, right] track half-widths over u = s / arclength.

        The CSV widths are measured from the CSV's own pixel-quantised centerline,
        so the walls are rebuilt in cartesian from it and measured again from the
        smoothed centerline.
        """
        nx, ny = self._normals(self.centerline_s_norm)
        normals = np.column_stack([nx, ny])
        left_wall = self.centerline + self.racetrack_widths[:, [0]] * normals
        right_wall = self.centerline - self.racetrack_widths[:, [1]] * normals

        s_grid = np.linspace(0.0, self.arclength, self.num_points, endpoint=False)
        widths = []
        for wall, sign in ((left_wall, 1.0), (right_wall, -1.0)):
            sd = self.convert_to_frenet_arr(wall)
            order = np.argsort(sd[:, 0])
            widths.append(sign * np.interp(s_grid, sd[order, 0], sd[order, 1], period=self.arclength))
        left, right = widths

        u = np.append(s_grid, self.arclength) / self.arclength
        tck_widths, _ = splprep([np.append(left, left[0]), np.append(right, right[0])], u=u,
                                s=self.num_points * 1e-4, k=3, per=True)
        return tck_widths

    def _normals(self, s_norm):
        dx, dy = splev(s_norm, self.centerline_spline, der=1)
        mag = np.hypot(dx, dy)
        return -dy / mag, dx / mag   # left-pointing

    def _get_normal(self, s_norm):
        return np.array(self._normals(s_norm))

    def convert_to_cartesian(self, s, d):
        s_norm = np.mod(s / self.arclength, 1.0)
        cx, cy = splev(s_norm, self.centerline_spline, der=0)
        centerline_point = np.array([cx, cy])
        normal = self._get_normal(s_norm)
        return centerline_point + d * normal

    def convert_to_frenet(self, x, y):
        return self.convert_to_frenet_arr([[x, y]])[0]

    def convert_to_cartesian_arr(self, s_arr, d_arr):
        """Vectorized convert_to_cartesian.

        Parameters
        ----------
        s_arr : (N,) array-like  – arc-length positions along the centerline (wrapped at arclength)
        d_arr : (N,) array-like  – signed lateral offsets (positive = left)

        Returns
        -------
        xy : (N, 2) ndarray  – Cartesian [x, y] for each (s, d) pair
        """
        s_arr = np.asarray(s_arr, dtype=float)
        d_arr = np.asarray(d_arr, dtype=float)

        s_norm = np.mod(s_arr / self.arclength, 1.0)

        cx, cy = splev(s_norm, self.centerline_spline, der=0)
        nx, ny = self._normals(s_norm)

        x = cx + d_arr * nx
        y = cy + d_arr * ny
        return np.column_stack([x, y])

    def convert_to_frenet_arr(self, xy_arr):
        """Vectorized convert_to_frenet: closest-point projection onto the centerline spline.

        Parameters
        ----------
        xy_arr : (N, 2) array-like  – Cartesian [x, y] query points

        Returns
        -------
        sd : (N, 2) ndarray  – Frenet [s, d_signed] for each input point, s in [0, arclength)
        """
        xy_arr = np.asarray(xy_arr, dtype=float).reshape(-1, 2)

        _, idx = self._lookup_tree.query(xy_arr)
        r = xy_arr - self._lookup_xy[idx]
        tx, ty = self._lookup_tangent[idx, 0], self._lookup_tangent[idx, 1]
        kappa = self._lookup_kappa[idx]
        along = r[:, 0] * tx + r[:, 1] * ty
        across = -r[:, 0] * ty + r[:, 1] * tx

        # Expanding the centerline about the nearest sample to second order
        # (C(s_i + h) ~ C_i + T h + kappa N h^2 / 2), the foot of the perpendicular
        # is at h = along / (1 - kappa * across). Clamped because past the centre of
        # curvature (kappa * d >= 1) the projection isn't unique.
        h = along / np.maximum(1.0 - kappa * across, 0.2)
        s = np.mod(self._lookup_s[idx] + h, self.arclength)
        d_signed = across - 0.5 * kappa * h**2 * (1.0 - kappa * across)

        return np.column_stack([s, d_signed])


    def curvature(self, s_norm):
        """Signed centerline curvature at normalized arc length s_norm in [0, 1].
        Positive for a left (CCW) turn. Accepts scalar or array."""
        dx, dy = splev(s_norm, self.centerline_spline, der=1)
        ddx, ddy = splev(s_norm, self.centerline_spline, der=2)
        num = dx * ddy - dy * ddx
        denom = (dx * dx + dy * dy) ** 1.5
        return num / denom

    def _offset_points(self, s_norm, d):
        cx, cy = splev(s_norm, self.centerline_spline, der=0)
        nx, ny = self._normals(s_norm)
        return cx + d * nx, cy + d * ny

    def region_polyline(self, s, ds, d, num_points=20):
        s_values = np.linspace(s, s + ds, num_points)
        s_norm = (s_values / self.arclength) % 1.0
        # x, y = self._offset_points(s_norm, d)
        kappa = self.curvature(s_norm)
        d_col = np.full(num_points, d)
        return np.column_stack([s_values, d_col, kappa])

    def polyline_set(self, s, ds, num_offsets=5, num_points=20, scale=0.9):
        s_values = np.linspace(s, s + ds, num_points)
        s_norm = (s_values / self.arclength) % 1.0
        left_widths, right_widths = splev(s_norm, self.racetrack_widths_spline, der=0)
        max_d = np.min(left_widths)
        min_d = -np.min(right_widths)
        d_values = np.linspace(scale * min_d, scale * max_d, num_offsets)
        polylines = [self.region_polyline(s, ds, d, num_points=num_points) for d in d_values]
        return np.stack(polylines)

    # ------------------------------------------------------------------
    # Whole-track precomputed fan (for context encoding)
    # ------------------------------------------------------------------
    def precompute_fan(self, num_points_total=1000, num_offsets=5, scale=0.9):
        """Dense fan of constant-offset polylines spanning the WHOLE track.

        Returns
        -------
        fan : (num_offsets, num_points_total, 3) float32
            Per-point columns [s_abs, d, offset_curvature]. The curvature stored
            is the *offset-path* curvature kappa / (1 - kappa*d), NOT the
            centerline curvature, so the offsets carry genuinely different
            geometry (the inside line through a corner is tighter than the
            outside line). This is what makes a 5-line fan worth encoding.
        d_values : (num_offsets,) float
            The constant offset used for each row.

        The offsets are GLOBAL: bounds come from the tightest half-width over
        the whole track (scaled by `scale`), so every offset stays in bounds
        everywhere and the fan is a uniform, index-able table sampled at
        `num_points_total` stations over [0, arclength).
        """
        s_grid = np.linspace(0.0, self.arclength, num_points_total, endpoint=False)
        s_norm = s_grid / self.arclength
        kappa = self.curvature(s_norm)                      # centerline curvature, shape (N,)

        left_w, right_w = splev(s_norm, self.racetrack_widths_spline, der=0)
        max_d = scale * float(np.min(left_w))               # tightest left half-width
        min_d = -scale * float(np.min(right_w))             # tightest right half-width (negative)
        d_values = np.linspace(min_d, max_d, num_offsets)

        fan = np.empty((num_offsets, num_points_total, 3), dtype=np.float32)
        for i, d in enumerate(d_values):
            offset_kappa = kappa / (1.0 - kappa * d)        # curvature of the offset path
            fan[i, :, 0] = s_grid
            fan[i, :, 1] = d
            fan[i, :, 2] = offset_kappa
        return fan, d_values

    def in_bounds_frenet(self, s, d):
        s_norm = np.mod(s / self.arclength, 1.0)
        left_width, right_width = splev(s_norm, self.racetrack_widths_spline, der=0)
        return -right_width < d < left_width

    def in_bounds_cartesian(self, x, y):
        s, d = self.convert_to_frenet(x, y)
        return self.in_bounds_frenet(s, d)

    def nearest_neighbor_cartesian(self, x, y):
        _, idx = self.centerline_tree.query([x, y])
        return self.centerline[idx]

    def nearest_neighbor_frenet(self, s, d):
        return np.array([s, 0.0])

    def start_point_cartesian(self):
        return self.centerline[0]

    def metadata(self):
        return {
            "num_points": self.num_points,
            "arclength": self.arclength,
            "start_point": self.centerline[0],
        }
