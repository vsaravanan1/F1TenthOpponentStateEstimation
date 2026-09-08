#!/usr/bin/env python3

import csv
import signal
import sys
from datetime import datetime

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class IMM_RMSE_EVAL(Node):
    def __init__(self):
        super().__init__('imm_rmse_eval')

        # --- subscriptions ---
        # IMM: full predicted path; we only use poses[0] (current-state prediction)
        self.imm_sub = self.create_subscription(Path, '/imm_path', self.path_callback, 10)
        # ground-truth opponent odometry
        self.odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_callback, 10)
        # LSTM: current-state prediction as a Marker (we use pose.position)
        self.lstm_sub = self.create_subscription(Marker, '/predicted_opponent_state', self.marker_callback, 10)

        # kept for live monitoring
        self.rmse_pub = self.create_publisher(String, '/rmse_error', 10)

        # --- ground-truth odom buffer ---
        self.last_odom_cb_time = self.get_clock().now()
        self.odom_buffer: list[dict] = []
        self.odom_cb_interval: Duration = Duration(seconds=0, nanoseconds=int(5e7))  # throttle to ~20 Hz
        self.odom_buf_max_len: int = 50

        # --- per-second squared-error accumulators (one sample per new model observation) ---
        self.imm_sq_errors: list[float] = []
        self.lstm_sq_errors: list[float] = []

        # --- CSV setup: one file per model, timestamped so replays don't clobber ---
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.imm_csv_file = open(f'imm_error_{stamp}.csv', 'w', newline='')
        self.lstm_csv_file = open(f'lstm_error_{stamp}.csv', 'w', newline='')
        self.imm_writer = csv.writer(self.imm_csv_file)
        self.lstm_writer = csv.writer(self.lstm_csv_file)
        header = ['time_sec', 'rmse_m', 'num_samples']
        self.imm_writer.writerow(header)
        self.lstm_writer.writerow(header)
        self.imm_csv_file.flush()
        self.lstm_csv_file.flush()

        # --- flush an averaged (RMSE) error to CSV once per second ---
        self.csv_timer = self.create_timer(1.0, self.csv_timer_cb, None, self.get_clock())

    # ------------------------------------------------------------------ #
    # ground-truth odom buffer
    # ------------------------------------------------------------------ #
    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now()
        if current_time - self.last_odom_cb_time < self.odom_cb_interval:
            return
        self.last_odom_cb_time = current_time
        if len(self.odom_buffer) >= self.odom_buf_max_len:
            self.odom_buffer.pop(0)
        self.odom_buffer.append({"time": current_time, "pose": msg.pose.pose.position})

    def _closest_odom(self, t):
        """Buffered odom position closest in time to t, or None if buffer empty."""
        if not self.odom_buffer:
            return None
        durations = [abs((t - o["time"]).nanoseconds) for o in self.odom_buffer]
        return self.odom_buffer[int(np.argmin(durations))]["pose"]

    @staticmethod
    def _sq_planar_error(a, b) -> float:
        """Squared planar (x, y) error between two geometry_msgs Points."""
        return (a.x - b.x) ** 2 + (a.y - b.y) ** 2

    # ------------------------------------------------------------------ #
    # IMM: poses[0] (current-state prediction) vs closest ground-truth odom
    # ------------------------------------------------------------------ #
    def path_callback(self, msg: Path):
        if not msg.poses:
            return
        pred = msg.poses[0].pose.position          # current-state prediction
        truth = self._closest_odom(self.get_clock().now())
        if truth is None:
            return
        self.imm_sq_errors.append(self._sq_planar_error(pred, truth))

    # ------------------------------------------------------------------ #
    # LSTM: marker pose.position vs latest ground-truth odom
    # ------------------------------------------------------------------ #
    def marker_callback(self, msg: Marker):
        if not self.odom_buffer:
            return
        pred = msg.pose.position                   # current-state prediction (.x, .y)
        truth = self.odom_buffer[-1]["pose"]       # latest odom
        self.lstm_sq_errors.append(self._sq_planar_error(pred, truth))

    # ------------------------------------------------------------------ #
    # once per second: write RMSE over the window, only if that model
    # produced new observations during the last second
    # ------------------------------------------------------------------ #
    def csv_timer_cb(self):
        t_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.imm_sq_errors:
            n = len(self.imm_sq_errors)
            rmse = float(np.sqrt(np.mean(self.imm_sq_errors)))
            self.imm_writer.writerow([f'{t_sec:.3f}', f'{rmse:.6f}', n])
            self.imm_csv_file.flush()
            self.get_logger().info(f'[IMM]  RMSE = {rmse:.4f} m over {n} samples')
            self.rmse_pub.publish(String(data=f'IMM RMSE: {rmse:.4f} m ({n} samples)\n'))
            self.imm_sq_errors.clear()

        if self.lstm_sq_errors:
            n = len(self.lstm_sq_errors)
            rmse = float(np.sqrt(np.mean(self.lstm_sq_errors)))
            self.lstm_writer.writerow([f'{t_sec:.3f}', f'{rmse:.6f}', n])
            self.lstm_csv_file.flush()
            self.get_logger().info(f'[LSTM] RMSE = {rmse:.4f} m over {n} samples')
            self.rmse_pub.publish(String(data=f'LSTM RMSE: {rmse:.4f} m ({n} samples)\n'))
            self.lstm_sq_errors.clear()

    def close_files(self):
        for f in (getattr(self, 'imm_csv_file', None), getattr(self, 'lstm_csv_file', None)):
            try:
                if f is not None:
                    f.flush()
                    f.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = IMM_RMSE_EVAL()

    def shutdown(*_):
        node.get_logger().info("\nBag finished or interrupted")
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    rclpy.spin(node)


if __name__ == "__main__":
    main()