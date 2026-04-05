#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.optimize import minimize_scalar
from scipy.special import expit
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class IMMInterceptorNode(Node):
    '''changed to sigmoid function and added offset from main trajectory'''

    def __init__(self):
        super().__init__('imm_interceptor')

        self.imm_path_sub = self.create_subscription(Path, '/imm_path', self.imm_path_callback, 10)
        self.ego_state_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_state_callback, 10)
        self.lidar_scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_scan_callback, 10)

        self.interceptor_pub = self.create_publisher(Path, '/interceptor_spline', 10)
        self.intercept_marker_pub = self.create_publisher(Marker, '/intercept_point_marker', 10)

        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_vx = 0.0
        self.ego_vy = 0.0
        self.ego_yaw = 0.0

        self.length_of_the_car = 0.5
        self.car_width = 0.30
        self.opp_half_width = 0.20
        self.clearance_margin = 0.35
        self.min_lateral_offset = 0.75

        self.dt = 0.05
        self.path_resolution = 45

        self.opp_path = None

        self.scan_ranges = np.array([])
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        self.max_scan_range = 4.0

    def lidar_scan_callback(self, msg: LaserScan):
        processed = []
        rmin = max(0.0, msg.range_min)
        rmax = msg.range_max if math.isfinite(msg.range_max) else self.max_scan_range
        cap = min(self.max_scan_range, rmax)
        for r in msg.ranges:
            if not math.isfinite(r) or r <= rmin:
                processed.append(0.0)
            else:
                processed.append(min(float(r), cap))
        self.scan_ranges = np.array(processed, dtype=np.float64)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def ego_state_callback(self, msg: Odometry):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = msg.twist.twist.linear.x
        self.ego_vy = msg.twist.twist.linear.y

        q = msg.pose.pose.orientation
        self.ego_yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

    def imm_path_callback(self, path_msg: Path):
        if len(path_msg.poses) == 0:
            return

        opponent_trajectory = np.array([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in path_msg.poses
        ], dtype=np.float64)

        self.opp_path = opponent_trajectory

        pass_target, side, index = self.find_optimal_pass_target(opponent_trajectory)
        if pass_target is None:
            self.get_logger().warn("[interceptor.py debug] No safe pass target found")
            return

        self.publish_intercept_marker(pass_target)
        self.gen_pub_sigmoid_path(pass_target, index, side)

    def find_optimal_pass_target(self, opponent_trajectory: np.ndarray):
        ego_pos = np.array([self.ego_x, self.ego_y], dtype=np.float64)
        ego_speed = max(0.5, np.hypot(self.ego_vx, self.ego_vy))

        def candidate_cost(t_index):
            index = int(np.clip(t_index, 2, len(opponent_trajectory) - 3))
            left_pt, right_pt, tangent = self.build_side_candidates(opponent_trajectory, index)

            opp_time = index * self.dt
            left_time = np.linalg.norm(left_pt - ego_pos) / ego_speed
            right_time = np.linalg.norm(right_pt - ego_pos) / ego_speed

            left_clear = self.estimate_side_clearance("left")
            right_clear = self.estimate_side_clearance("right")

            left_forward = np.dot(left_pt - ego_pos, tangent)
            right_forward = np.dot(right_pt - ego_pos, tangent)

            left_cost = (left_time - opp_time) ** 2 - 0.35 * left_clear - 0.15 * left_forward
            right_cost = (right_time - opp_time) ** 2 - 0.35 * right_clear - 0.15 * right_forward
            return min(left_cost, right_cost)

        result = minimize_scalar(
            candidate_cost,
            bounds=(2, len(opponent_trajectory) - 3),
            method='bounded'
        )

        best_index = int(np.clip(result.x, 2, len(opponent_trajectory) - 3))
        left_pt, right_pt, tangent = self.build_side_candidates(opponent_trajectory, best_index)
        left_clear = self.estimate_side_clearance("left")
        right_clear = self.estimate_side_clearance("right")

        left_score = self.score_candidate(left_pt, tangent, left_clear)
        right_score = self.score_candidate(right_pt, tangent, right_clear)

        if left_score >= right_score:
            chosen_side = "left"
            chosen_pt = left_pt
        else:
            chosen_side = "right"
            chosen_pt = right_pt

        return chosen_pt, chosen_side, best_index

    def build_side_candidates(self, opponent_trajectory: np.ndarray, index: int):
        prev_pt = opponent_trajectory[index - 2]
        curr_pt = opponent_trajectory[index]
        next_pt = opponent_trajectory[index + 2]

        tangent = next_pt - prev_pt
        norm_tan = np.linalg.norm(tangent)
        tangent = tangent / norm_tan

        normal_left = np.array([-tangent[1], tangent[0]], dtype=np.float64)
        normal_right = -normal_left

        lateral_offset = max(
            self.min_lateral_offset,
            0.5 * self.car_width + self.opp_half_width + self.clearance_margin
        )

        forward_lead = max(0.3, self.length_of_the_car)
        base_anchor = curr_pt + forward_lead * tangent

        left_pt = base_anchor + lateral_offset * normal_left
        right_pt = base_anchor + lateral_offset * normal_right
        return left_pt, right_pt, tangent

    def score_candidate(self, candidate: np.ndarray, tangent: np.ndarray, side_clearance: float):
        ego_pos = np.array([self.ego_x, self.ego_y], dtype=np.float64)
        ego_speed = max(0.5, np.hypot(self.ego_vx, self.ego_vy))

        dist = np.linalg.norm(candidate - ego_pos)
        travel_time = dist / ego_speed
        forward_gain = np.dot(candidate - ego_pos, tangent)
        return 0.6 * side_clearance + 0.2 * forward_gain - 0.2 * travel_time

    def estimate_side_clearance(self, side: str) -> float:
        if self.scan_ranges.size == 0:
            return 0.0

        if side == "left":
            amin = math.radians(10.0)
            amax = math.radians(70.0)
        else:
            amin = math.radians(-70.0)
            amax = math.radians(-10.0)

        values = []
        for i, r in enumerate(self.scan_ranges):
            if r <= 0.0:
                continue
            ang = self.scan_angle_min + i * self.scan_angle_inc
            if amin <= ang <= amax:
                values.append(r)

        if not values:
            return 0.0
        return float(min(values))

    def gen_pub_sigmoid_path(self, pass_target: np.ndarray, target_index: int, side: str):
        ego_pos = np.array([self.ego_x, self.ego_y], dtype=np.float64)

        if self.opp_path is None or len(self.opp_path) < 5:
            return

        prev_pt = self.opp_path[max(0, target_index - 2)]
        next_pt = self.opp_path[min(len(self.opp_path) - 1, target_index + 2)]
        tangent = next_pt - prev_pt
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm < 1e-6:
            tangent = np.array([math.cos(self.ego_yaw), math.sin(self.ego_yaw)], dtype=np.float64)
        else:
            tangent = tangent / tangent_norm

        if side == "left":
            normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
        else:
            normal = np.array([tangent[1], -tangent[0]], dtype=np.float64)

        lateral_offset = max(
            self.min_lateral_offset,
            0.5 * self.car_width + self.opp_half_width + self.clearance_margin
        )

        opp_anchor = self.opp_path[target_index]
        ahead_point = opp_anchor + 1.3 * tangent + lateral_offset * normal

        # local coordinates in frame aligned with pass direction
        delta_end = ahead_point - ego_pos
        dlong = max(1.0, np.dot(delta_end, tangent))
        dlat = np.dot(delta_end, normal)

        if abs(dlat) < 1e-3:
            dlat = lateral_offset if side == "left" else -lateral_offset

        # paper-inspired sigmoid parameters of the research paper found 
        # center shifted slightly forward so lane change starts after a short straight segment
        center = 0.45 * dlong
        sharpness = 10.0 / dlong

        x_samples = np.linspace(0.0, dlong, self.path_resolution)
        raw = expit(sharpness * (x_samples - center))

        # normalize so path starts exactly at 0 and ends exactly at dlat
        raw0 = expit(sharpness * (0.0 - center))
        raw1 = expit(sharpness * (dlong - center))
        denom = max(1e-6, raw1 - raw0)
        y_samples = dlat * (raw - raw0) / denom

        world_points = []
        for x_local, y_local in zip(x_samples, y_samples):
            pt = ego_pos + x_local * tangent + y_local * normal
            world_points.append(pt)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pt in world_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.interceptor_pub.publish(path_msg)
        self.get_logger().info(
            f"Published sigmoid overtake path with {len(path_msg.poses)} points on {side} side"
        )

    def publish_intercept_marker(self, target_point: np.ndarray):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "intercept"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(target_point[0])
        marker.pose.position.y = float(target_point[1])
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        self.intercept_marker_pub.publish(marker)

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = IMMInterceptorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
