#!/usr/bin/env python3
import numpy as np
import pandas as pd
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev


class RacetrackUtilities:
    def __init__(self, csv_file):
        self.centerline_tree = None
        self.centerline = None
        self.racetrack_widths = None

        centerline_shape = self.parse_csv(csv_file)
        self.num_points = centerline_shape[0]

        u, tck, arclength, tck_widths = self.create_centerline_splines()
        self.centerline_s_norm = u
        self.centerline_spline = tck
        self.racetrack_widths_spline = tck_widths
        self.arclength = arclength

    def parse_csv(self, csv_file):
        raceline_df = pd.read_csv(csv_file)
        self.centerline = raceline_df[["x_m", "y_m"]].to_numpy()
        self.racetrack_widths = raceline_df[["w_tr_left_m", "w_tr_right_m"]].to_numpy()
        self.centerline_tree = KDTree(self.centerline)
        return self.centerline.shape

    def create_centerline_splines(self):
        segment_lengths = np.linalg.norm(np.diff(self.centerline, axis=0), axis=1)
        cumlen = np.concatenate([[0], np.cumsum(segment_lengths)])
        total_arclength = cumlen[-1]
        u = cumlen / total_arclength

        tck, _ = splprep([self.centerline[:, 0], self.centerline[:, 1]], u=u, s=0, k=3, per=True)
        tck_widths, _ = splprep([self.racetrack_widths[:, 0], self.racetrack_widths[:, 1]], u=u, s=0, k=3, per=True)
        return u, tck, total_arclength, tck_widths

    def _get_normal(self, s_norm):
        dx, dy = splev(s_norm, self.centerline_spline, der=1)
        tangent = np.array([dx, dy])
        tangent = tangent / np.linalg.norm(tangent)
        normal = np.array([-tangent[1], tangent[0]])
        return normal

    def convert_to_cartesian(self, s, d):
        s_norm = np.clip(s / self.arclength, 0.0, 1.0)
        cx, cy = splev(s_norm, self.centerline_spline, der=0)
        centerline_point = np.array([cx, cy])
        normal = self._get_normal(s_norm)
        return centerline_point + d * normal

    def convert_to_frenet(self, x, y):
        query_point = np.array([x, y])
        d, idx = self.centerline_tree.query(query_point)
        s_norm = self.centerline_s_norm[idx]
        s = s_norm * self.arclength
        normal = self._get_normal(s_norm)
        diff = query_point - self.centerline[idx]
        d_signed = np.sign(np.dot(diff, normal)) * d
        return np.array([s, d_signed])

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
        dx, dy = splev(s_norm, self.centerline_spline, der=1)
        mag = np.hypot(dx, dy)
        nx = -dy / mag
        ny = dx / mag
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
        s_norm = np.clip(s / self.arclength, 0.0, 1.0)
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