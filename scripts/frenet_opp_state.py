#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from racetrack_utilities.racetrack_utilities import RacetrackUtilities


class FrenetOpponentStateNode(Node):
    def __init__(self):
        super().__init__('FrenetOpponentStateNode')

        name = "/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv"
        self.racetrack = RacetrackUtilities(name)

        meta = self.racetrack.metadata()
        self.get_logger().info(f"Loaded racetrack: {meta['num_points']} points, {meta['arclength']:.1f}m")

        self.last_publish_time = None

        self.state_sub = self.create_subscription(
            Float64MultiArray, "/state_vector", self.state_cb, 10
        )
        self.frenet_pub = self.create_publisher(
            Float64MultiArray, "/frenet_opp_state_vector", 10
        )

    def state_cb(self, msg: Float64MultiArray):
        # unpack — ignore vx, vy
        _, x, y, _vx, _vy = msg.data

        # bounds check
        if not self.racetrack.in_bounds_cartesian(x, y):
            self.get_logger().debug(f"({x:.2f}, {y:.2f}) outside racetrack bounds — skipping")
            return

        s, d = self.racetrack.convert_to_frenet(x, y)

        # wall proximity check
        D_THRESHOLD = 0.9
        if d > D_THRESHOLD or d < -D_THRESHOLD:
            self.get_logger().debug(f"d={d:.2f} too close to wall — skipping")
            return

        # compute elapsed time
        now = self.get_clock().now()
        if self.last_publish_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_publish_time).nanoseconds * 1e-9
        self.last_publish_time = now

        # publish [s, d, dt]
        out = Float64MultiArray()
        out.data = [s, d, dt]
        self.frenet_pub.publish(out)
        self.get_logger().info(f"dt={dt:.4f}s  s={s:.2f}m  d={d:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    node = FrenetOpponentStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()