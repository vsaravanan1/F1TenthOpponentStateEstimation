#!/usr/bin/python3
"""
io_utils.py  (dynamic MPPI)

Self-contained helpers so the dynamic package does not import from f1tenth_mppi:
  * load_waypoints      - read the Spielberg sparse CSV into [x, y, psi, vx]
  * pack_waypoints      - np (N, M) -> std_msgs/Float64MultiArray  ("csv-like")
  * unpack_waypoints    - Float64MultiArray -> np (N, M)
  * line_marker / sphere_markers - small viz helpers
"""

import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def load_waypoints(path):
    """
    Load the Spielberg centerline CSV (semicolon-separated, '#'-commented):
        s; x; y; psi; kappa; vx; ax
    Returns (N, 4) = [x, y, psi, vx]  (psi in the CSV/TUM convention, vx raw m/s).
    The node normalises vx into the throttle range afterwards, exactly like the
    original node. If your real load_waypoints differs, swap this out.
    """
    raw = np.genfromtxt(path, delimiter=";", comments="#")
    if raw.ndim != 2 or raw.shape[1] < 6:
        raise ValueError(f"unexpected waypoint CSV shape {raw.shape} in {path}")
    return np.stack([raw[:, 1], raw[:, 2], raw[:, 3], raw[:, 5]], axis=1)


def pack_waypoints(arr):
    """np (N, M) -> Float64MultiArray, row-major, with rows/cols in the layout."""
    arr = np.asarray(arr, dtype=float)
    n, m = arr.shape
    msg = Float64MultiArray()
    d0 = MultiArrayDimension(label="rows", size=n, stride=n * m)
    d1 = MultiArrayDimension(label="cols", size=m, stride=m)
    msg.layout.dim = [d0, d1]
    msg.layout.data_offset = 0
    msg.data = arr.flatten().tolist()
    return msg


def unpack_waypoints(msg):
    """Float64MultiArray -> np (N, M). Falls back to 2 cols if layout is missing."""
    data = np.asarray(msg.data, dtype=float)
    if len(msg.layout.dim) >= 2 and msg.layout.dim[1].size > 0:
        m = int(msg.layout.dim[1].size)
    else:
        m = 2
    n = len(data) // m
    return data[: n * m].reshape(n, m)


def line_marker(points_xy, ns, mid, rgb, frame="map", width=0.04, stamp=None):
    mk = Marker()
    mk.header.frame_id = frame
    if stamp is not None:
        mk.header.stamp = stamp
    mk.ns = ns
    mk.id = mid
    mk.type = Marker.LINE_STRIP
    mk.action = Marker.ADD
    mk.scale.x = width
    mk.color.r, mk.color.g, mk.color.b, mk.color.a = float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0
    mk.pose.orientation.w = 1.0
    for p in points_xy:
        pt = Point(); pt.x = float(p[0]); pt.y = float(p[1]); pt.z = 0.0
        mk.points.append(pt)
    return mk