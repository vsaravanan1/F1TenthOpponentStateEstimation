#!/usr/bin/env python3

import csv
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray


class SplineLogger(Node):
    """Subscribes to the ego and opponent spline topics, keeps the latest
    MarkerArray from each, and every `log_period` seconds appends the points
    of the currently-held splines (with timestamps) to a CSV file."""

    def __init__(self):
        super().__init__('spline_logger')

        # ---- parameters -------------------------------------------------
        self.declare_parameter('csv_path', '/sim_ws/src/lidar_processing/scripts/spline_log.csv')
        self.declare_parameter('log_period', 0.5)
        self.declare_parameter('num_opponents', 3)
        # Log all ego candidates; switch to '/ego_lane_possibilities' to log
        # only the chosen best defence instead.
        self.declare_parameter('ego_topic', '/ego_candidate_splines')

        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.log_period = self.get_parameter('log_period').get_parameter_value().double_value
        self.num_opponents = self.get_parameter('num_opponents').get_parameter_value().integer_value
        self.ego_topic = self.get_parameter('ego_topic').get_parameter_value().string_value

        # ---- latest-message storage ------------------------------------
        # Only the newest MarkerArray per source is retained.
        self._lock = threading.Lock()
        self.latest_ego = None
        self.latest_opp = {i: None for i in range(self.num_opponents)}

        # Match the publishers' default QoS (RELIABLE, KEEP_LAST).
        qos = QoSProfile(depth=10,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(MarkerArray, self.ego_topic,
                                 self.ego_cb, qos)
        for i in range(self.num_opponents):
            # idx=i binds the current value as a default arg
            self.create_subscription(
                MarkerArray, f'/predicted_opponent_splines_{i}',
                lambda msg, idx=i: self.opp_cb(msg, idx), qos)

        # ---- CSV file --------------------------------------------------
        self.fieldnames = ['log_time', 'source', 'marker_id',
                           'point_index', 'x', 'y', 'marker_stamp']
        new_file = (not os.path.exists(self.csv_path)) or os.path.getsize(self.csv_path) == 0
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        if new_file:
            self.writer.writeheader()
            self.csv_file.flush()

        self.create_timer(self.log_period, self.log_timer_cb)
        self.get_logger().info(
            f'Logging splines to {os.path.abspath(self.csv_path)} '
            f'every {self.log_period}s')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def ego_cb(self, msg: MarkerArray):
        with self._lock:
            self.latest_ego = msg

    def opp_cb(self, msg: MarkerArray, idx: int):
        with self._lock:
            self.latest_opp[idx] = msg

    # ------------------------------------------------------------------
    # Timer: snapshot the current splines to CSV
    # ------------------------------------------------------------------
    def log_timer_cb(self):
        with self._lock:
            ego = self.latest_ego
            opp = dict(self.latest_opp)

        log_time = self.get_clock().now().nanoseconds * 1e-9

        rows = self._marker_array_rows(ego, 'ego', log_time)
        for i in range(self.num_opponents):
            rows += self._marker_array_rows(opp.get(i), f'opponent_{i}', log_time)

        if not rows:
            return

        self.writer.writerows(rows)
        self.csv_file.flush()

    def _marker_array_rows(self, marker_array, source, log_time):
        """One row per (spline point) in the MarkerArray."""
        rows = []
        if marker_array is None:
            return rows
        for marker in marker_array.markers:
            stamp = marker.header.stamp.sec + marker.header.stamp.nanosec * 1e-9
            for j, pt in enumerate(marker.points):
                rows.append({
                    'log_time': f'{log_time:.6f}',
                    'source': source,
                    'marker_id': marker.id,
                    'point_index': j,
                    'x': f'{pt.x:.6f}',
                    'y': f'{pt.y:.6f}',
                    'marker_stamp': f'{stamp:.6f}',
                })
        return rows

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SplineLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()