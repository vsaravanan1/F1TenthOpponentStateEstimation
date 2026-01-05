#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_follow_gap')

        # Topics
        self.lidarscan_topic = "/opp_scan"
        self.drive_topic = "/opp_drive"

        # Speed parameters
        self.MAX_SPEED = 3.0                 # Top speed for straight driving with clear path
        self.HIGH_SPEED = 2.0               # minor turns or limited clearance
        self.MEDIUM_SPEED = 1.0             # caution speed, moderate turns or low clearance
        self.DEF_LOW_SPEED = 0.8            # default else low speed
        # evasive speed is defined in # Evasive Manuever section

        # Perception parameters
        self.MAX_VALID_RANGE = 5.0              # cap far readings (m)
        self.FREE_RANGE_THRESHOLD = 0.5         # min range to consider free (m)
        self.BUBBLE_RADIUS = 150                # indices around closest obstacle
        self.DISPARITY_THRESHOLD = 0.5          # adjacent range difference (m)
        self.CAR_WIDTH = 0.30                   # vehicle width (m)
        self.EXTEND_SCALE = 1.2                 # disparity inflation scaling
        self.ROI_HALF_ANGLE = math.radians(90)  # consider only ±90° around front

        # Control parameters
        self.TURN_SIDE_SECTOR = math.radians(60) # forward sector to check on turning side
        self.CLEARANCE_MARGIN = 0.5           # m; min side clearance while turning
        self.MAX_STEER_DELTA = math.radians(2)   # rad per callback; steering rate limit

        # Inside/outside wall avoidance (symmetric)
        self.LEFT_CLEARANCE_TARGET = 0.8         # m; desired min clearance on the left
        self.RIGHT_CLEARANCE_TARGET = 0.8         # m; desired min clearance on the right
        self.LEFT_SECTOR_ANGLE = math.radians(70)
        self.RIGHT_SECTOR_ANGLE = math.radians(70)
        self.LEFT_REPULSION_GAIN = 0.8           # rad per meter of deficit
        self.RIGHT_REPULSION_GAIN = 0.8           # rad per meter of deficit

        # Evasive maneuver (turn away from whichever side is too close)
        self.EVASIVE_THRESHOLD = 0.5           # m; trigger evasive
        self.EVASIVE_STEER = math.radians(15)     # rad; target turn in evasive
        self.EVASIVE_SPEED = 0.6                 # m/s; low speed while evading

        # State
        self.processed_lidar = []
        self.prev_steer = 0.0

        # ROS I/O
        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.subscription_ = self.create_subscription(LaserScan, self.lidarscan_topic, self.lidar_callback, 10)

    # ---------------- Perception pipeline ----------------

    def preprocess_lidar(self, scan_msg):
        """Clamp far values and drop invalids."""
        self.processed_lidar = []
        rmin = max(0.0, scan_msg.range_min)
        rmax = scan_msg.range_max if math.isfinite(scan_msg.range_max) else self.MAX_VALID_RANGE
        cap = min(self.MAX_VALID_RANGE, rmax)
        for r in scan_msg.ranges:
            if not math.isfinite(r) or r <= rmin:
                self.processed_lidar.append(0.0)
            else:
                self.processed_lidar.append(min(float(r), cap))

    def apply_front_roi(self, scan_msg):
        """Mask rays outside forward sector ±ROI_HALF_ANGLE."""
        if not self.processed_lidar:
            return
        amin, ainc = scan_msg.angle_min, scan_msg.angle_increment
        n = len(self.processed_lidar)
        roi_min = -self.ROI_HALF_ANGLE
        roi_max = +self.ROI_HALF_ANGLE

        def angle_to_index(a):
            return int(max(0, min(n - 1, math.floor((a - amin) / max(ainc, 1e-6)))))

        start_i = angle_to_index(roi_min)
        end_i = angle_to_index(roi_max)
        if end_i <= start_i:
            return
        for i in range(0, start_i):
            self.processed_lidar[i] = 0.0
        for i in range(end_i + 1, n):
            self.processed_lidar[i] = 0.0

    def apply_safety_bubble(self):
        """Zero-out indices around the closest obstacle."""
        if not self.processed_lidar:
            return
        min_dist = float('inf')
        closest_i = 0
        for i, d in enumerate(self.processed_lidar):
            if 0.01 < d < min_dist:
                min_dist = d
                closest_i = i
        if not math.isfinite(min_dist) or min_dist == float('inf'):
            return
        lo = max(0, closest_i - self.BUBBLE_RADIUS)
        hi = min(len(self.processed_lidar), closest_i + self.BUBBLE_RADIUS)
        for i in range(lo, hi):
            self.processed_lidar[i] = 0.0

    def apply_disparity_extender(self, scan_msg):
        """Inflate corners: mask wedges formed by large adjacent disparities."""
        ranges = self.processed_lidar
        if not ranges:
            return
        inc = scan_msg.angle_increment
        n = len(ranges)
        extended = ranges[:]  # copy

        for i in range(n - 1):
            r1, r2 = ranges[i], ranges[i + 1]
            if r1 <= 0.0 or r2 <= 0.0:
                continue
            if abs(r1 - r2) > self.DISPARITY_THRESHOLD:
                closer = min(r1, r2)
                half_w = self.CAR_WIDTH / 2.0
                ang = math.atan2(half_w, max(closer, 1e-3))  # rad
                n_extend = max(1, int(self.EXTEND_SCALE * ang / max(inc, 1e-6)))

                if r1 < r2:
                    start = i + 1
                    end = min(n, start + n_extend)
                    for k in range(start, end):
                        extended[k] = 0.0
                else:
                    start = max(0, i - n_extend + 1)
                    end = i + 1
                    for k in range(start, end):
                        extended[k] = 0.0

        self.processed_lidar = extended

    def find_max_gap(self):
        """Return (start_index, length) of longest run of free space."""
        largest_starting_i = 0
        longest_gap = 0
        curr_gap = 0
        for i in range(len(self.processed_lidar)):
            if self.processed_lidar[i] < self.FREE_RANGE_THRESHOLD:
                curr_gap = 0
            else:
                curr_gap += 1
                if curr_gap > longest_gap:
                    longest_gap = curr_gap
                    largest_starting_i = i - curr_gap + 1
        return largest_starting_i, longest_gap

    def find_best_point(self, starting_i, gap_length):
        """Choose farthest point within the gap (bias away from walls)."""
        # end_i = starting_i + gap_length
        # best_i = starting_i
        # best_r = -1.0
        # for i in range(starting_i, end_i):
        #     r = self.processed_lidar[i]
        #     if r > best_r:
        #         best_r = r
        #         best_i = i
        return starting_i + gap_length//2

    # ---------------- Steering shaping ----------------

    def clamp_turn_by_clearance(self, steering_angle, scan_msg):
        """Reduce steering magnitude if turning-side clearance is below margin."""
        if not self.processed_lidar:
            return steering_angle, 0.0
        inc = scan_msg.angle_increment
        i0 = int(round((0.0 - scan_msg.angle_min) / max(inc, 1e-6)))
        i0 = max(0, min(len(self.processed_lidar) - 1, i0))

        n_sector = max(1, int(self.TURN_SIDE_SECTOR / max(inc, 1e-6)))
        if steering_angle >= 0.0:
            start = i0
            end = min(len(self.processed_lidar) - 1, i0 + n_sector)
        else:
            start = max(0, i0 - n_sector)
            end = i0

        sector_vals = [v for v in self.processed_lidar[start:end + 1] if v > 0.0]
        min_clearance = min(sector_vals) if sector_vals else 0.0

        if min_clearance < self.CLEARANCE_MARGIN and abs(steering_angle) > 1e-3:
            scale = max(0.2, min_clearance / max(self.CLEARANCE_MARGIN, 1e-6))
            steering_angle = math.copysign(scale * abs(steering_angle), steering_angle)

        return steering_angle, min_clearance

    def measure_side_clearances(self, scan_msg):
        """Measure minimum clearances on left and right forward sectors."""
        if not self.processed_lidar:
            return 0.0, 0.0
        inc = scan_msg.angle_increment
        n = len(self.processed_lidar)
        i0 = int(round((0.0 - scan_msg.angle_min) / max(inc, 1e-6)))
        i0 = max(0, min(n - 1, i0))

        n_left = max(1, int(self.LEFT_SECTOR_ANGLE / max(inc, 1e-6)))
        n_right = max(1, int(self.RIGHT_SECTOR_ANGLE / max(inc, 1e-6)))

        # Left sector: [i0, i0 + n_left]
        l_start = i0
        l_end = min(n - 1, i0 + n_left)
        left_vals = [v for v in self.processed_lidar[l_start:l_end + 1] if v > 0.0]
        left_min = min(left_vals) if left_vals else 0.0

        # Right sector: [i0 - n_right, i0]
        r_start = max(0, i0 - n_right)
        r_end = i0
        right_vals = [v for v in self.processed_lidar[r_start:r_end + 1] if v > 0.0]
        right_min = min(right_vals) if right_vals else 0.0

        return left_min, right_min

    def apply_side_repulsion(self, steering_angle, left_min, right_min):
        """Push away from close walls on either side."""
        # Left too close -> steer right (negative)
        left_deficit = max(0.0, self.LEFT_CLEARANCE_TARGET - left_min)
        left_offset = self.LEFT_REPULSION_GAIN * left_deficit

        # Right too close -> steer left (positive)
        right_deficit = max(0.0, self.RIGHT_CLEARANCE_TARGET - right_min)
        right_offset = self.RIGHT_REPULSION_GAIN * right_deficit

        # Combine: rightward is negative, leftward is positive
        steering_angle = steering_angle - left_offset + right_offset
        return steering_angle

    def apply_evasive_if_needed(self, steering_angle, left_min, right_min, min_clearance):
        """If a side is below threshold, turn away from that side to avoid hitting the wall"""
        left_bad = left_min < self.EVASIVE_THRESHOLD
        right_bad = right_min < self.EVASIVE_THRESHOLD

        if left_bad and right_bad:
            # Turn away from the worse side
            if left_min <= right_min:
                return -self.EVASIVE_STEER, True  # steer right
            else:
                return +self.EVASIVE_STEER, True  # steer left
        if left_bad:
            return -self.EVASIVE_STEER, True
        if right_bad:
            return +self.EVASIVE_STEER, True

        # Turning-side clearance (min_clearance) still too low: turn away from current turn
        if min_clearance < self.EVASIVE_THRESHOLD:
            return (-self.EVASIVE_STEER if steering_angle >= 0.0 else +self.EVASIVE_STEER), True

        return steering_angle, False

    def rate_limit_steering(self, steering_angle):
        """Limit change per callback to avoid abrupt turn-ins or oscillations."""
        delta = steering_angle - self.prev_steer
        if abs(delta) > self.MAX_STEER_DELTA:
            steering_angle = self.prev_steer + math.copysign(self.MAX_STEER_DELTA, delta)
        self.prev_steer = steering_angle
        return steering_angle

    # ---------------- Main callback ----------------

    def lidar_callback(self, scan_msg):
        # 1) Perception
        self.preprocess_lidar(scan_msg)
        self.apply_front_roi(scan_msg)
        self.apply_safety_bubble()
        self.apply_disparity_extender(scan_msg)

        # 2) Gap selection
        gap_start, gap_len = self.find_max_gap()
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()

        if gap_len == 0:
            self.get_logger().warning("No gap found! Emergency stop.")
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.0
            self.publisher_.publish(drive_msg)
            return

        best_i = self.find_best_point(gap_start, gap_len)
        steering_angle = scan_msg.angle_min + best_i * scan_msg.angle_increment

        # 3) Side measurements and steering shaping
        left_min, right_min = self.measure_side_clearances(scan_msg)
        steering_angle = self.apply_side_repulsion(steering_angle, left_min, right_min)
        steering_angle, min_clearance = self.clamp_turn_by_clearance(steering_angle, scan_msg)
        steering_angle, evasive = self.apply_evasive_if_needed(steering_angle, left_min, right_min, min_clearance)
        steering_angle = self.rate_limit_steering(steering_angle)

        drive_msg.drive.steering_angle = steering_angle

        # 4) Speed policy
        steering_abs = abs(steering_angle)
        target_range = self.processed_lidar[best_i] if 0 <= best_i < len(self.processed_lidar) else 0.0

        if evasive:
            speed = self.EVASIVE_SPEED
        elif target_range < 0.20:  # hard stop only for an obstacle directly ahead
            speed = 0.0
        elif steering_abs < math.radians(10):
            speed = self.MAX_SPEED if target_range > 1.0 and min_clearance > 0.5 else self.HIGH_SPEED
        elif steering_abs < math.radians(20):
            speed = self.HIGH_SPEED if target_range > 0.8 and min_clearance > 0.4 else self.MEDIUM_SPEED
        else:
            speed = self.DEF_LOW_SPEED

        drive_msg.drive.speed = speed
        self.publisher_.publish(drive_msg)

        self.get_logger().info(
            f"steer={steering_angle:.3f} rad, speed={speed:.2f} m/s, min_clear={min_clearance:.2f} m, "
            f"left_min={left_min:.2f} m, right_min={right_min:.2f} m, evasive={'Y' if evasive else 'N'}"
        )

# main
def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap (Python) Initialized")
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()