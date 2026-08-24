#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import String

class OvertakeInterceptorNode(Node):
    def __init__(self):
        super().__init__('overtake_interceptor')
        
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/opp_scan', self.scan_callback, 10)
        self.opp_odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_callback, 10)
        self.imm_sub = self.create_subscription(Path, '/imm_path_original', self.imm_path_callback, 10)
        
        self.overtake_path_pub = self.create_publisher(Path, '/overtake_spline', 10)
        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)
        self.marker_pub = self.create_publisher(Marker, '/overtake_marker', 10)
        
        self.ego_x = self.ego_y = self.ego_yaw = self.ego_speed = 0.0
        self.opp_x = self.opp_y = self.opp_yaw = self.opp_speed = 0.0
        
        self.ego_data_received = False
        self.opp_data_received = False
        self.imm_data_received = False
        
        self.scan_ranges = np.array([])
        self.scan_angle_min = self.scan_angle_inc = 0.0
        
        self.car_width = 0.30
        self.lateral_clearance = 1
        self.overtake_distance = 6.0
        self.path_resolution = 50
        
        # FIX 1: Per your intuition, drop the distance so it drafts tight before popping out!
        self.max_display_distance = 3.5   
        self.min_ego_distance = 0.0
        
        self.current_mode = "FTG"
        self.current_side = "left"
        self.locked_spline = None
        self.debug_counter = 0
        
        self.create_timer(0.05, self.continuous_spline_update)
        self.get_logger().info("🎯 PLANNER Started - AGGRESSIVE EVASION MATH ACTIVE!")

    def set_mode(self, new_mode):
        if new_mode != self.current_mode:
            self.get_logger().info(f"\n====================================\n🚦 PLANNER TRANSITION: {self.current_mode} ➡️ {new_mode}\n====================================")
            self.current_mode = new_mode
            self.mode_pub.publish(String(data=self.current_mode))

    def ego_odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.ego_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.ego_data_received = True

    def opp_odom_callback(self, msg):
        self.opp_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.opp_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if not self.imm_data_received:
            self.opp_x = msg.pose.pose.position.x
            self.opp_y = msg.pose.pose.position.y
        self.opp_data_received = True

    def imm_path_callback(self, msg):
        if len(msg.poses) > 0:
            self.opp_x = msg.poses[0].pose.position.x
            self.opp_y = msg.poses[0].pose.position.y
            self.imm_data_received = True

    def scan_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def continuous_spline_update(self):
        self.debug_counter += 1
        if not (self.ego_data_received and self.opp_data_received):
            return

        if not self.is_obstacle_ahead():
            self.publish_empty_path()
            self.locked_spline = None
            self.set_mode("FTG")
            if self.debug_counter % 20 == 0:
                self.get_logger().info("🥇 Obstacle cleared! Returning to FTG.")
            return

        if self.locked_spline is not None:
            self.publish_path(self.locked_spline)
            self.set_mode("OVERTAKE_ACTIVE")
            return

        ego_pos = np.array([self.ego_x, self.ego_y])
        opp_pos = np.array([self.opp_x, self.opp_y])
        distance = np.linalg.norm(opp_pos - ego_pos)
        
        if distance < self.max_display_distance and distance >= self.min_ego_distance:
            self.update_overtake_side()
            new_spline = self.generate_safe_spline()
            
            if new_spline is not None:
                self.locked_spline = new_spline 
                self.publish_path(new_spline)
                self.set_mode("OVERTAKE_ACTIVE")
                self.publish_continuous_marker()
            else:
                self.publish_empty_path()
                self.set_mode("FTG")
                if self.debug_counter % 20 == 0:
                    self.get_logger().warning("🚫 Walls/Obstacles blocking pass. Awaiting clearance.")
        else:
            self.publish_empty_path()
            self.set_mode("FTG")

    def is_obstacle_ahead(self):
        dx = self.ego_x - self.opp_x
        dy = self.ego_y - self.opp_y
        hx = math.cos(self.opp_yaw)
        hy = math.sin(self.opp_yaw)
        return (dx * hx + dy * hy) > -1.5

    def update_overtake_side(self):
        opp_forward = np.array([math.cos(self.opp_yaw), math.sin(self.opp_yaw)])
        to_ego = np.array([self.ego_x - self.opp_x, self.ego_y - self.opp_y])
        cross_product = np.cross(opp_forward, to_ego)
        
        if self.current_side == "left" and cross_product < -0.8:
            self.current_side = "right"
        elif self.current_side == "right" and cross_product > 0.8:
            self.current_side = "left"
        elif self.current_side not in ["left", "right"]:
            self.current_side = "left" if cross_product > 0 else "right"

    def generate_safe_spline(self):
        spline = self.try_generate_spline(self.current_side)
        if spline is not None: return spline
        
        other_side = "right" if self.current_side == "left" else "left"
        spline = self.try_generate_spline(other_side)
        
        if spline is not None:
            self.current_side = other_side
            self.get_logger().info(f"🔄 Switched pass to {self.current_side.upper()} due to wall blockage!")
            return spline
        return None

    def try_generate_spline(self, side):
        opp_pos = np.array([self.opp_x, self.opp_y])
        ego_pos = np.array([self.ego_x, self.ego_y])
        
        opp_forward = np.array([math.cos(self.opp_yaw), math.sin(self.opp_yaw)])
        ego_forward = np.array([math.cos(self.ego_yaw), math.sin(self.ego_yaw)])
        
        if side == "left":
            lateral_dir = np.array([-ego_forward[1], ego_forward[0]])
        else:
            lateral_dir = np.array([ego_forward[1], -ego_forward[0]])
        
        adaptive_distance = min(self.overtake_distance, 8.0)
        end_point = ego_pos + adaptive_distance * ego_forward + self.lateral_clearance * lateral_dir
        
        dist_to_ego = np.linalg.norm(ego_pos - opp_pos)
        dist_total = np.linalg.norm(end_point - opp_pos)
        
        if dist_total < 0.5:
            return None
            
        # ---------------- AGGRESSIVE EVASION BEZIER MATH ----------------
        P0 = opp_pos
        
        # P1: Only push forward 20% (was 60%). Forces immediate steering into passing lane!
        P1 = opp_pos + (dist_to_ego * 0.2) * opp_forward
        
        P3 = end_point
        
        # P2: Pull backward 80% (was 60%). Drags the curve into passing lane BEFORE reaching target!
        P2 = P3 - (self.overtake_distance * 0.8) * ego_forward
        
        path_points = []
        t_values = np.linspace(0, 1, self.path_resolution)
        
        for t in t_values:
            point = ((1 - t)**3) * P0 + \
                    3 * ((1 - t)**2) * t * P1 + \
                    3 * (1 - t) * (t**2) * P2 + \
                    (t**3) * P3
            path_points.append(point)
            
        return self.validate_and_clip_path(path_points)

    def validate_and_clip_path(self, path_points):
        if len(self.scan_ranges) == 0:
            return path_points
            
        for pt in path_points:
            dx = pt[0] - self.opp_x
            dy = pt[1] - self.opp_y
            dist = math.hypot(dx, dy)
            
            local_angle = math.atan2(dy, dx) - self.opp_yaw
            local_angle = (local_angle + math.pi) % (2 * math.pi) - math.pi
            
            idx = int(round((local_angle - self.scan_angle_min) / max(self.scan_angle_inc, 1e-6)))
            if 0 <= idx < len(self.scan_ranges):
                r = self.scan_ranges[idx]
                if math.isfinite(r) and r > 0.1 and dist > (r - self.car_width - 0.15):
                    return None
                    
        return np.array(path_points)

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, point in enumerate(path_points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.1
            
            if i < len(path_points) - 1:
                yaw = math.atan2(path_points[i+1][1] - point[1], path_points[i+1][0] - point[0])
            else:
                yaw = 0.0
                
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)
            
        self.overtake_path_pub.publish(path_msg)

    def publish_empty_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.overtake_path_pub.publish(path_msg)

    def publish_continuous_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "overtake_origin"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.opp_x)
        marker.pose.position.y = float(self.opp_y)
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        if self.current_side == "left":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(marker)

    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def main(args=None):
    rclpy.init(args=args)
    node = OvertakeInterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()