#!/usr/bin/python3
"""
divert_injector_node.py

TEST tool: lets you "pipe in" a divert spline at runtime so you can watch the
dynamic MPPI follow it and return to the centerline -- no planner needed.

It reads a CSV of path points and publishes them on `out_topic` (same message the
MPPI consumes) when triggered. Two CSV frames are supported:

  frame = "car"  (default): CSV is x-forward / y-left relative to the car. On trigger
                 the points are transformed to the map frame at the car's CURRENT
                 pose, so the divert always starts from the car -- exactly like the
                 real planner, which anchors the defensive spline at the ego.
  frame = "map": CSV is already in map coordinates; published as-is.

CSV columns: x,y   (optional 3rd column = normalised throttle; else divert_speed).
'#'-commented header lines are ignored.

Trigger options (any of):
  * publish once, `startup_delay` s after launch (if auto_fire = true)
  * send std_msgs/Empty on `trigger_topic`:
        ros2 topic pub --once /inject_divert std_msgs/msg/Empty "{}"
  * swap CSV live:
        ros2 param set /divert_injector csv_path <...>/divert_right.csv
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
from nav_msgs.msg import Odometry

from f1tenth_mppi_dynamic.io_utils import pack_waypoints


class DivertInjector(Node):
    def __init__(self):
        super().__init__("divert_injector")
        self.declare_parameters("", [
            ("csv_path", ""),
            ("frame", "car"),                 # "car" or "map"
            ("out_topic", "/divert_waypoints"),
            ("pose_topic", "/ego_racecar/odom"),
            ("trigger_topic", "/inject_divert"),
            ("divert_speed", 0.85),
            ("auto_fire", True),
            ("startup_delay", 3.0),
        ])
        self.pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("out_topic").value, 10)
        self.pose_sub = self.create_subscription(
            Odometry, self.get_parameter("pose_topic").value, self.pose_cb, 10)
        self.trig_sub = self.create_subscription(
            Empty, self.get_parameter("trigger_topic").value, lambda m: self.fire(), 10)
        self.car = None
        if self.get_parameter("auto_fire").value:
            self.create_timer(self.get_parameter("startup_delay").value, self._auto_once)
        self._fired_auto = False
        self.get_logger().info("divert_injector ready; trigger with "
                               f"`ros2 topic pub --once {self.get_parameter('trigger_topic').value} "
                               "std_msgs/msg/Empty \"{}\"`")

    def pose_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        self.car = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _auto_once(self):
        if not self._fired_auto:
            self._fired_auto = True
            self.fire()

    def fire(self):
        path = self.get_parameter("csv_path").value
        if not path:
            self.get_logger().error("csv_path not set")
            return
        try:
            rel = np.genfromtxt(path, delimiter=",", comments="#")
        except Exception as e:
            self.get_logger().error(f"could not read {path}: {e}")
            return
        if rel.ndim != 2 or rel.shape[1] < 2:
            self.get_logger().error(f"CSV {path} must have >=2 columns (x,y)")
            return

        xy = rel[:, :2]
        if self.get_parameter("frame").value == "car":
            if self.car is None:
                self.get_logger().warn("no pose yet; cannot place a car-frame divert")
                return
            cx, cy, cyaw = self.car
            R = np.array([[math.cos(cyaw), -math.sin(cyaw)],
                          [math.sin(cyaw), math.cos(cyaw)]])
            xy = np.array([cx, cy]) + xy @ R.T

        v = rel[:, 2] if rel.shape[1] >= 3 else np.full(len(xy), self.get_parameter("divert_speed").value)
        # publish [x, y, v] -> MPPI fills psi from geometry (xy+? ) ; to keep MPPI
        # geometry-free we send [x,y] only and let the (light) converter path run in
        # MPPI, OR send xy here and let MPPI build psi. We send [x,y] (2 cols).
        out = np.stack([xy[:, 0], xy[:, 1]], axis=1)
        self.pub.publish(pack_waypoints(out))
        self.get_logger().info(f"injected divert from {path} ({len(out)} pts, "
                               f"frame={self.get_parameter('frame').value})")


def main(args=None):
    rclpy.init(args=args)
    node = DivertInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()