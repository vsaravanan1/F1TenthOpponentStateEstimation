#!/usr/bin/env python3
import numpy as np
import pandas as pd
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev


"""
Raceline utilities class performs the following functionality:
1. Parses raceline csv with columns: x_m, y_m, w_tr_right_m, w_tr_left_m to create numpy arrays representing centerline coordinates and track widths
2. Converts coordinates from (x, y) to (s, d) and vice versa
3. Returns total arclength of raceline
4. Returns if a coordinate is within the bounds of the racetrack
5. Returns nearest neighbor to arbitrary point (input can be in frenet or cartesian frames, so can the output)
6. Returns metadata about csv file (column names, number of points, etc.)
7. Returns starting point in cartesian coordinates
"""
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
        """Returns unit normal vector (pointing left) at normalized arc length s_norm."""
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


    def in_bounds_frenet(self, s, d):
        s_norm = np.clip(s / self.arclength, 0.0, 1.0)
        left_width, right_width = splev(s_norm, self.racetrack_widths_spline, der=0)
        return -right_width < d < left_width


    def in_bounds_cartesian(self, x, y):
        s, d = self.convert_to_frenet(x, y)
        return self.in_bounds_frenet(s, d)


    def nearest_neighbor_cartesian(self, x, y):
        """Returns nearest centerline point in cartesian coordinates."""
        _, idx = self.centerline_tree.query([x, y])
        return self.centerline[idx]


    def nearest_neighbor_frenet(self, s, d):
        """Returns nearest centerline point in frenet coordinates (d=0 by definition)."""
        return np.array([s, 0.0])


    def start_point_cartesian(self):
        return self.centerline[0]


    def metadata(self):
        return {
            "num_points": self.num_points,
            "arclength": self.arclength,
            "start_point": self.centerline[0],
        }