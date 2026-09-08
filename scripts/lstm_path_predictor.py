#!/usr/bin/env python3
"""
ROS2 node that predicts and visualizes the opponent vehicle's future
trajectory using the raceline-context LSTM seq2seq model.

Model interface (must match lstm.py / the training script):

    Inputs per history step:
        [opp_s, opp_d, ego_s, ego_d, ego_v]        # 5 features, NO dt

    Context (per prediction):
        a fan of constant-offset polylines over [s_start, s_end], extracted
        from a precomputed whole-track fan via get_polyline_segments(...)

    Outputs per future step:
        [delta_s, delta_d]                          # BOTH are deltas
"""

import math
from collections import deque

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, ColorRGBA
from visualization_msgs.msg import Marker

from racetrack_utilities.racetrack_utilities import RacetrackUtilities

# Import the model + context helper from the (import-safe) training module.
from lstm import (
    LSTMModel,
    get_polyline_segments,
    NUM_OFFSETS,
    FAN_RESOLUTION,
)


# -----------------------------
# Constants that must match training
# -----------------------------
HISTORY_LEN = 15
FUTURE_LEN = 40
N_FEATURES = 5          # [opp_s, opp_d, ego_s, ego_d, ego_v]  (dt removed)
N_HIDDEN = 51
N_CONTEXT = 32          # must equal n_context used in training

# Training history spacing. Real observations are resampled to this cadence.
TARGET_DT = 0.05        # seconds (50 ms)
# Cap sub-steps from a single gap so a pathological dt can't spin forever;
# more than HISTORY_LEN sub-steps would just refill the whole deque anyway.
MAX_INTERP_STEPS = HISTORY_LEN

STALE_POSITION_EPS = 1e-6


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Create a yaw-only quaternion without requiring tf2."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class LSTMOpponentPathPredictorNode(Node):
    def __init__(self):
        super().__init__('LSTMOpponentPathPredictorNode')

        # ---- parameters ----
        self.declare_parameter(
            'map_csv_path',
            '/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv',
        )
        self.declare_parameter(
            'model_path',
            '/sim_ws/src/lidar_processing/scripts/checkpoint/'
            'singleposbestmodel.pt',
        )
        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('opp_frenet_topic', '/frenet_opp_state_vector')
        self.declare_parameter('predicted_marker_topic', '/predicted_opponent_state')
        self.declare_parameter('no_observation_timeout', 2.00)
        self.declare_parameter('pseudo_update_interval', 0.750)

        map_csv_path = self.get_parameter('map_csv_path').value
        model_path = self.get_parameter('model_path').value
        ego_odom_topic = self.get_parameter('ego_odom_topic').value
        opp_frenet_topic = self.get_parameter('opp_frenet_topic').value
        predicted_marker_topic = self.get_parameter('predicted_marker_topic').value
        self.no_observation_timeout = float(
            self.get_parameter('no_observation_timeout').value
        )
        self.pseudo_update_interval = float(
            self.get_parameter('pseudo_update_interval').value
        )

        # ---- racetrack ----
        self.racetrack = RacetrackUtilities(map_csv_path)
        meta = self.racetrack.metadata()
        self.track_length = float(meta['arclength'])
        self.get_logger().info(
            f"Loaded racetrack: {meta['num_points']} points, {self.track_length:.1f}m"
        )

        # ---- precomputed context fan (built once, same params as training) ----
        fan_np, d_values = self.racetrack.precompute_fan(
            num_points_total=FAN_RESOLUTION, num_offsets=NUM_OFFSETS
        )
        self.fan = torch.from_numpy(fan_np)          # (P, N, 3), cpu float32
        self.fan_arclength = float(self.racetrack.arclength)
        self.get_logger().info(
            f"Precomputed context fan: {tuple(self.fan.shape)}, "
            f"offsets={np.round(d_values, 3)}"
        )

        # ---- model ----
        self.model = LSTMModel(
            n_hidden=N_HIDDEN, n_context=N_CONTEXT, num_offsets=NUM_OFFSETS
        )
        state_dict = torch.load(model_path, map_location='cpu')
        self.model.load_state_dict(state_dict)   # strict: fails loud on width mismatch
        self.model.eval()
        self.get_logger().info(f"Loaded LSTM weights from {model_path}")

        # ---- current ego state ----
        self.latest_ego_s_wrapped = None
        self.latest_ego_s = None
        self.latest_ego_d = None
        self.latest_ego_v = None

        # ---- opponent/model state ----
        # Each history entry (training feature order, 5 features):
        # [opp_s_unwrapped, opp_d, ego_s_unwrapped, ego_d, ego_v]
        self.history = deque(maxlen=HISTORY_LEN)

        # Elapsed time from skipped stale measurements, folded into the next
        # accepted observation so interpolation spans the true interval.
        self.pending_dt = 0.0

        self.last_prediction = None      # [FUTURE_LEN, 2] absolute [opp_s_unwrapped, opp_d]
        self.last_real_obs_time = None
        self.in_pseudo_mode = False

        # ---- pub/sub ----
        self.ego_odom_sub = self.create_subscription(
            Odometry, ego_odom_topic, self.ego_odom_cb, 10
        )
        self.opp_frenet_sub = self.create_subscription(
            Float64MultiArray, opp_frenet_topic, self.opp_frenet_cb, 10
        )
        self.marker_pub = self.create_publisher(Marker, predicted_marker_topic, 10)
        # self.pseudo_update_timer = self.create_timer(
        #     self.pseudo_update_interval, self.pseudo_update_cb
        # )

    # -------------------------------------------------
    def unwrap_s(self, wrapped_s: float, reference_unwrapped_s):
        """Unwrap one Frenet s measurement around a continuous reference."""
        wrapped_s = float(wrapped_s) % self.track_length
        if reference_unwrapped_s is None:
            return wrapped_s
        reference_wrapped = reference_unwrapped_s % self.track_length
        half = self.track_length / 2.0
        delta = ((wrapped_s - reference_wrapped + half) % self.track_length) - half
        return float(reference_unwrapped_s + delta)

    # -------------------------------------------------
    def ego_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not self.racetrack.in_bounds_cartesian(x, y):
            self.get_logger().debug(
                f"ego ({x:.2f}, {y:.2f}) outside racetrack bounds -- skipping"
            )
            return

        ego_s_wrapped, ego_d = self.racetrack.convert_to_frenet(x, y)
        ego_vx = msg.twist.twist.linear.x
        ego_vy = msg.twist.twist.linear.y
        ego_v = math.sqrt(ego_vx ** 2 + ego_vy ** 2)

        self.latest_ego_s = self.unwrap_s(ego_s_wrapped, self.latest_ego_s)
        self.latest_ego_s_wrapped = float(ego_s_wrapped)
        self.latest_ego_d = float(ego_d)
        self.latest_ego_v = float(ego_v)

    # -------------------------------------------------
    def reset_prediction_state(self):
        """Clear state when a new overtake/teleport instance begins."""
        self.history.clear()
        self.pending_dt = 0.0
        self.last_prediction = None
        self.last_real_obs_time = None
        self.in_pseudo_mode = False

        if self.latest_ego_s_wrapped is not None:
            self.latest_ego_s = self.latest_ego_s_wrapped
        else:
            self.latest_ego_s = None

        self.get_logger().info("Cleared LSTM history for new overtake instance")

    # -------------------------------------------------
    def opp_frenet_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn(
                "Opponent Frenet message must contain [opp_s, opp_d, dt]"
            )
            return

        opp_s_wrapped, opp_d, dt = msg.data[:3]
        dt = float(dt)

        # dt <= 0 is the teleport/reset signal (dt itself is not a model input).
        if dt <= 0.0:
            self.reset_prediction_state()
            dt = 0.0

        if self.in_pseudo_mode:
            self.get_logger().info(
                "Real opponent observation received -- exiting pseudo mode"
            )
            self.in_pseudo_mode = False

        self.last_real_obs_time = self.get_clock().now()

        if (
            self.latest_ego_s is None
            or self.latest_ego_d is None
            or self.latest_ego_v is None
        ): return

        reference_opp_s = self.history[-1][0] if len(self.history) > 0 else None
        opp_s_unwrapped = self.unwrap_s(opp_s_wrapped, reference_opp_s)
        opp_d = float(opp_d)

        self._append_point(opp_s_unwrapped, opp_d, self.latest_ego_s, self.latest_ego_d, self.latest_ego_v, dt)
        laps = self.history[-1][0] // self.track_length
        if laps >= 2:
            shift = laps * self.track_length
            for row in self.history:
                row[0] -= shift          # opp_s
                row[2] -= shift          # ego_s
            if self.last_prediction is not None:
                self.last_prediction[:, 0] -= shift
                
        if len(self.history) >= HISTORY_LEN:
            self.predict_and_publish()
        


    # -------------------------------------------------
    def _append_point(self, opp_s, opp_d, ego_s, ego_d, ego_v, dt):
        self.history.append([
            float(opp_s), float(opp_d),
            float(ego_s), float(ego_d), float(ego_v), float(dt)
        ])


    # -------------------------------------------------
    def pseudo_update_cb(self):
        """Advance the model using its own absolute predicted trajectory when the
        opponent has not been observed for a while."""
        if self.last_real_obs_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_real_obs_time).nanoseconds * 1e-9
        if elapsed < self.no_observation_timeout:
            return
        if self.last_prediction is None:
            return
        if (self.latest_ego_s is None or self.latest_ego_d is None
                or self.latest_ego_v is None):
            return

        if not self.in_pseudo_mode:
            self.get_logger().warn(
                f"No opponent observation for {elapsed:.2f}s "
                f"(timeout={self.no_observation_timeout:.2f}s) -- "
                f"switching to pseudo updates every "
                f"{self.pseudo_update_interval:.2f}s"
            )
            self.in_pseudo_mode = True

        # last_prediction is already at TARGET_DT spacing; step 0 is one 50 ms
        # step ahead of the current anchor, so push it directly (no interp).
        pseudo_opp_s, pseudo_opp_d = self.last_prediction[0]
        self._append_point(
            float(pseudo_opp_s), float(pseudo_opp_d),
            self.latest_ego_s, self.latest_ego_d, self.latest_ego_v, float(dt)
        )
        if len(self.history) >= HISTORY_LEN:
            self.predict_and_publish()

    # -------------------------------------------------
    def predict_and_publish(self):
        # History as (HISTORY_LEN, 5): [opp_s, opp_d, ego_s, ego_d, ego_v]
        x_raw = np.asarray(self.history, dtype=np.float32)
        x_tensor = torch.from_numpy(x_raw).unsqueeze(0)          # (1, 15, 5)

        # Context window, computed EXACTLY as in training. Now that history is
        # resampled to TARGET_DT, the (now - prev) step is a true 50 ms delta,
        # so this constant-velocity s_end is correctly scaled.
        opp_s_now = float(x_raw[-1, 0])
        opp_s_prev = float(x_raw[-2, 0])
        s_start = float(x_raw[0, 0])
        s_end = opp_s_now + FUTURE_LEN * (opp_s_now - opp_s_prev)

        segments = get_polyline_segments(
            self.fan,
            self.fan_arclength,
            torch.tensor([s_start]),
            torch.tensor([s_end]),
        )                                                        # (1, P, 20, 3)

        with torch.no_grad():
            pred_delta = self.model(x_tensor, segments)

        pred_delta = pred_delta.cpu().numpy()         # (1, 2) - maybe

        # Outputs are cumulative [delta_s, delta_d] from the final history
        # opponent position. Reconstruct absolute continuous Frenet positions.
        anchor_opp_s = float(x_raw[-1, 0])
        anchor_opp_d = float(x_raw[-1, 1])

        pred_abs = np.empty_like(pred_delta)
        pred_abs[0, 0] = anchor_opp_s + pred_delta[0, 0]
        pred_abs[0, 1] = anchor_opp_d + pred_delta[0, 1]

        self.last_prediction = pred_abs
        print(pred_abs)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "opponent_state_estimate"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        x, y = self.racetrack.convert_to_cartesian(float(pred_abs[0,0] % self.racetrack.arclength), pred_abs[0, 1])
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
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
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = LSTMOpponentPathPredictorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()