#!/usr/bin/python3
"""
path_converter_node.py

PRODUCTION bridge: subscribes to the defensive spline the planner publishes as a
visualization_msgs/MarkerArray (map-frame LINE_STRIP points), converts it to the
"csv-like" waypoints the dynamic MPPI consumes, and republishes on `out_topic`.

This is the "path conversion algorithm" that keeps the heavy per-message work OUT
of the MPPI node. MPPI just tracks what arrives.

If the selected defensive path is published on a different topic/type (e.g. a
nav_msgs/Path), only the subscription + point-extraction below need to change.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from f1tenth_mppi_dynamic.reference_manager import ReferencePlanner
from f1tenth_mppi_dynamic.io_utils import pack_waypoints


class PathConverter(Node):
    def __init__(self):
        super().__init__("divert_path_converter")
        self.declare_parameters("", [
            ("in_topic", "/ego_lane_possibilities"),
            ("out_topic", "/divert_waypoints"),
            ("divert_speed", 0.85),     # normalised throttle for the divert
            ("spacing", 0.1),           # resample spacing (m)
            ("min_points", 3),
        ])
        self.sub = self.create_subscription(
            MarkerArray, self.get_parameter("in_topic").value, self.cb, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("out_topic").value, 10)
        self.get_logger().info(
            f"converter: {self.get_parameter('in_topic').value} -> "
            f"{self.get_parameter('out_topic').value}")

    def cb(self, msg: MarkerArray):
        if not msg.markers:
            return
        # choose the marker with the most points (the selected defensive path)
        mk = max(msg.markers, key=lambda m: len(m.points))
        if len(mk.points) < self.get_parameter("min_points").value:
            return
        xy = np.array([[p.x, p.y] for p in mk.points], dtype=float)
        wps = ReferencePlanner.path_to_waypoints(
            xy, self.get_parameter("divert_speed").value,
            self.get_parameter("spacing").value)
        self.pub.publish(pack_waypoints(wps))
        self.get_logger().info(f"published divert: {len(wps)} waypoints")


def main(args=None):
    rclpy.init(args=args)
    node = PathConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()