#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class EgoFTGController(Node):
    def __init__(self):
        super().__init__('ego_ftg_controller')
        
        # Subscriptions and Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Control state
        self.scan_msg = None
        self.processed_lidar = []
        self.prev_steer = 0.0
        
        # ---------------- FTG TUNED PARAMETERS (Baseline Speeds) ----------------
        self.MAX_SPEED = 1.67       # Top speed for straight driving
        self.HIGH_SPEED = 1.20      # Minor turns or limited clearance
        self.MEDIUM_SPEED = 1.00    # Caution speed, moderate turns
        self.DEF_LOW_SPEED = 0.80   # Default low speed
        self.EVASIVE_SPEED = 0.50   # Hard avoidance speed
        
        # Perception parameters
        self.MAX_VALID_RANGE = 3.0 
        self.FREE_RANGE_THRESHOLD = 0.9 
        self.BUBBLE_RADIUS = 40 
        self.DISPARITY_THRESHOLD = 0.4 
        self.CAR_WIDTH = 0.30 
        self.EXTEND_SCALE = 1.2 
        self.ROI_HALF_ANGLE = math.radians(90)
        
        # Control parameters
        self.TURN_SIDE_SECTOR = math.radians(60) 
        self.CLEARANCE_MARGIN = 0.50 
        self.MAX_STEER_DELTA = math.radians(4) 
        self.MAX_STEER_ANGLE = math.radians(25)
        
        # Wall avoidance
        self.LEFT_CLEARANCE_TARGET = 0.8 
        self.RIGHT_CLEARANCE_TARGET = 0.8 
        self.LEFT_SECTOR_ANGLE = math.radians(70)
        self.RIGHT_SECTOR_ANGLE = math.radians(70)
        self.LEFT_REPULSION_GAIN = 1.6 
        self.RIGHT_REPULSION_GAIN = 1.6 
        
        # Evasive maneuver
        self.EVASIVE_THRESHOLD = 0.6 
        self.EVASIVE_STEER = math.radians(22)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🏎️ EGO FTG CONTROLLER Started (Baseline Speeds)!")

    def scan_callback(self, msg):
        self.scan_msg = msg

    # ---------------- MAIN CONTROL LOOP ----------------

    def control_loop(self):
        if self.scan_msg is None: return

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()

        self.preprocess_lidar(self.scan_msg)
        self.apply_front_roi(self.scan_msg)
        self.apply_safety_bubble()
        self.apply_disparity_extender(self.scan_msg)

        steering, speed = self.ftg_control_post_perception()

        # Final limits and rate clamping
        steering = self.rate_limit_steering(steering)
        steering = np.clip(steering, -self.MAX_STEER_ANGLE, self.MAX_STEER_ANGLE)
        
        drive_msg.drive.steering_angle = float(steering)
        drive_msg.drive.speed = float(max(0.0, speed))
        
        self.drive_pub.publish(drive_msg)

    # ---------------- FOLLOW THE GAP SAFETY LOGIC ----------------

    def ftg_control_post_perception(self):
        scan_msg = self.scan_msg
        
        gap_start, gap_len = self.find_max_gap()
        if gap_len == 0: return 0.0, 0.0
            
        best_i = self.find_best_point(gap_start, gap_len)
        steering_angle = scan_msg.angle_min + best_i * scan_msg.angle_increment
        
        left_min, right_min = self.measure_side_clearances(scan_msg)
        steering_angle = self.apply_side_repulsion(steering_angle, left_min, right_min)
        steering_angle, min_clearance = self.clamp_turn_by_clearance(steering_angle, scan_msg)
        steering_angle, evasive = self.apply_evasive_if_needed(steering_angle, left_min, right_min, min_clearance)
        
        steering_abs = abs(steering_angle)
        target_range = self.processed_lidar[best_i] if 0 <= best_i < len(self.processed_lidar) else 0.0
        
        if evasive: speed = self.EVASIVE_SPEED
        elif target_range < 0.20: speed = 0.0
        elif steering_abs < math.radians(10): speed = self.MAX_SPEED if target_range > 1.0 and min_clearance > 0.5 else self.HIGH_SPEED
        elif steering_abs < math.radians(20): speed = self.HIGH_SPEED if target_range > 0.8 and min_clearance > 0.4 else self.MEDIUM_SPEED
        else: speed = self.DEF_LOW_SPEED
            
        corridor = min(left_min, right_min)
        if corridor < 0.8: speed = min(speed, 0.80) 
            
        return steering_angle, speed

    def preprocess_lidar(self, scan_msg):
        self.processed_lidar = []
        rmin = max(0.0, scan_msg.range_min)
        rmax = scan_msg.range_max if math.isfinite(scan_msg.range_max) else self.MAX_VALID_RANGE
        cap = min(self.MAX_VALID_RANGE, rmax)
        for r in scan_msg.ranges: self.processed_lidar.append(0.0 if not math.isfinite(r) or r <= rmin else min(float(r), cap))

    def apply_front_roi(self, scan_msg):
        if not self.processed_lidar: return
        amin, ainc = scan_msg.angle_min, scan_msg.angle_increment
        n = len(self.processed_lidar)
        start_i = int(max(0, min(n - 1, math.floor((-self.ROI_HALF_ANGLE - amin) / max(ainc, 1e-6)))))
        end_i = int(max(0, min(n - 1, math.floor((self.ROI_HALF_ANGLE - amin) / max(ainc, 1e-6)))))
        if end_i <= start_i: return
        for i in range(0, start_i): self.processed_lidar[i] = 0.0
        for i in range(end_i + 1, n): self.processed_lidar[i] = 0.0

    def apply_safety_bubble(self):
        if not self.processed_lidar: return
        min_dist, closest_i = float('inf'), 0
        for i, d in enumerate(self.processed_lidar):
            if 0.01 < d < min_dist: min_dist, closest_i = d, i
        if not math.isfinite(min_dist): return
        lo = max(0, closest_i - self.BUBBLE_RADIUS)
        hi = min(len(self.processed_lidar), closest_i + self.BUBBLE_RADIUS)
        for i in range(lo, hi): self.processed_lidar[i] = 0.0

    def apply_disparity_extender(self, scan_msg):
        ranges = self.processed_lidar
        if not ranges: return
        inc = scan_msg.angle_increment
        n = len(ranges)
        extended = ranges[:] 
        for i in range(n - 1):
            r1, r2 = ranges[i], ranges[i + 1]
            if r1 <= 0.0 or r2 <= 0.0: continue
            if abs(r1 - r2) > self.DISPARITY_THRESHOLD:
                ang = math.atan2(self.CAR_WIDTH / 2.0, max(min(r1, r2), 1e-3))
                n_extend = max(1, int(self.EXTEND_SCALE * ang / max(inc, 1e-6)))
                start = i + 1 if r1 < r2 else max(0, i - n_extend + 1)
                end = min(n, start + n_extend) if r1 < r2 else i + 1
                for k in range(start, end): extended[k] = 0.0
        self.processed_lidar = extended

    def find_max_gap(self):
        largest_starting_i, longest_gap, curr_gap = 0, 0, 0
        for i in range(len(self.processed_lidar)):
            if self.processed_lidar[i] < self.FREE_RANGE_THRESHOLD: curr_gap = 0
            else:
                curr_gap += 1
                if curr_gap > longest_gap: longest_gap, largest_starting_i = curr_gap, i - curr_gap + 1
        return largest_starting_i, longest_gap

    def find_best_point(self, starting_i, gap_length):
        if gap_length <= 0: return starting_i
        mid = (starting_i + starting_i + gap_length - 1) // 2
        window = max(1, gap_length // 6) 
        lo, hi = max(starting_i, mid - window), min(starting_i + gap_length - 1, mid + window)
        best_i, best_r = mid, -1.0
        for i in range(lo, hi + 1):
            if self.processed_lidar[i] > best_r: best_r, best_i = self.processed_lidar[i], i
        return best_i

    def clamp_turn_by_clearance(self, steering_angle, scan_msg):
        if not self.processed_lidar: return steering_angle, 0.0
        i0 = max(0, min(len(self.processed_lidar) - 1, int(round((0.0 - scan_msg.angle_min) / max(scan_msg.angle_increment, 1e-6)))))
        n_sector = max(1, int(self.TURN_SIDE_SECTOR / max(scan_msg.angle_increment, 1e-6)))
        start, end = (i0, min(len(self.processed_lidar) - 1, i0 + n_sector)) if steering_angle >= 0.0 else (max(0, i0 - n_sector), i0)
        sector_vals = [v for v in self.processed_lidar[start:end + 1] if v > 0.0]
        min_clearance = min(sector_vals) if sector_vals else 0.0
        if min_clearance < self.CLEARANCE_MARGIN and abs(steering_angle) > 1e-3:
            steering_angle = math.copysign(max(0.2, min_clearance / max(self.CLEARANCE_MARGIN, 1e-6)) * abs(steering_angle), steering_angle)
        return steering_angle, min_clearance

    def measure_side_clearances(self, scan_msg):
        if not self.processed_lidar: return 0.0, 0.0
        inc = scan_msg.angle_increment
        i0 = max(0, min(len(self.processed_lidar) - 1, int(round((0.0 - scan_msg.angle_min) / max(inc, 1e-6)))))
        n_left = max(1, int(self.LEFT_SECTOR_ANGLE / max(inc, 1e-6)))
        n_right = max(1, int(self.RIGHT_SECTOR_ANGLE / max(inc, 1e-6)))
        left_vals = [v for v in self.processed_lidar[i0:min(len(self.processed_lidar) - 1, i0 + n_left) + 1] if v > 0.0]
        right_vals = [v for v in self.processed_lidar[max(0, i0 - n_right):i0 + 1] if v > 0.0]
        return (min(left_vals) if left_vals else 0.0), (min(right_vals) if right_vals else 0.0)

    def apply_side_repulsion(self, steering_angle, left_min, right_min):
        return steering_angle - (self.LEFT_REPULSION_GAIN * max(0.0, self.LEFT_CLEARANCE_TARGET - left_min)) + (self.RIGHT_REPULSION_GAIN * max(0.0, self.RIGHT_CLEARANCE_TARGET - right_min))

    def apply_evasive_if_needed(self, steering_angle, left_min, right_min, min_clearance):
        left_bad, right_bad = left_min < self.EVASIVE_THRESHOLD, right_min < self.EVASIVE_THRESHOLD
        if left_bad and right_bad: return (-self.EVASIVE_STEER if left_min <= right_min else +self.EVASIVE_STEER), True 
        if left_bad: return -self.EVASIVE_STEER, True
        if right_bad: return +self.EVASIVE_STEER, True
        if min_clearance < self.EVASIVE_THRESHOLD: return (-self.EVASIVE_STEER if steering_angle >= 0.0 else +self.EVASIVE_STEER), True
        return steering_angle, False

    def rate_limit_steering(self, steering_angle):
        delta = steering_angle - self.prev_steer
        if abs(delta) > self.MAX_STEER_DELTA: steering_angle = self.prev_steer + math.copysign(self.MAX_STEER_DELTA, delta)
        self.prev_steer = steering_angle
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    node = EgoFTGController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()