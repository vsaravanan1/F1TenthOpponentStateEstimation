#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from racetrack_utilities.racetrack_utilities import RacetrackUtilities

class CenterlinePublisherNode(Node):
    def __init__(self):
        super().__init__('CenterlinePublisherNode')

        name = "/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv"
        self.racetrack = RacetrackUtilities(name)

        meta = self.racetrack.metadata()
        self.get_logger().info(f"Loaded racetrack: {meta['num_points']} points, {meta['arclength']:.1f}m")

        self.published_path = self._build_path()

        self.raceline_publisher = self.create_publisher(Path, '/centerline', 10)
        self.raceline_publish_timer = self.create_timer(2, self.raceline_pub_cb)

        self.ego_odom_sub = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.ego_odom_cb, 10
        )

    def _build_path(self):
        path = Path()
        path.header.frame_id = "map"
        for point in self.racetrack.centerline:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path.poses.append(pose)
        return path

    def ego_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        s, d = self.racetrack.convert_to_frenet(x, y)

        if self.racetrack.in_bounds_frenet(s, d):
            self.get_logger().info(f"s={s:.2f}m  d={d:.2f}m")
        else:
            self.get_logger().warning(f"Out of bounds! s={s:.2f}m  d={d:.2f}m")

    def raceline_pub_cb(self):
        self.raceline_publisher.publish(self.published_path)


def main(args=None):
    rclpy.init(args=args)
    node = CenterlinePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()