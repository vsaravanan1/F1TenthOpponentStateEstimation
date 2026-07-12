#!/usr/bin/env python3
"""
ROS2 node that predicts and visualizes the opponent vehicle's future
trajectory using the pretrained LSTM seq2seq model from the training
script.

Pipeline
--------
1. Listen to ego odometry (/ego_racecar/odom). Convert ego (x, y) into
   Frenet coordinates (ego_s, ego_d) via RacetrackUtilities, and take
   ego_v as the magnitude of the ego linear velocity.
2. Listen to the opponent's Frenet state (/frenet_opp_state_vector),
   published as [opp_s, opp_d, dt] by FrenetOpponentStateNode.
3. Combine [opp_s, opp_d, ego_s, ego_d, ego_v] -- same feature order
   used in TrajectoryDataset during training -- into one feature
   vector per opponent observation, and push it into a fixed-size
   sliding window (a deque with maxlen=HISTORY_LEN automatically
   drops the oldest sample as new ones arrive).
4. Once the window holds HISTORY_LEN=5 samples, run one forward pass
   of the LSTM (history_len=5 -> future_len=10) to autoregressively
   predict the opponent's future [s, d] for the next 10 timesteps.
5. Convert each predicted (s, d) back to Cartesian map-frame (x, y)
   and publish the sequence as a nav_msgs/Path for RViz.

Partial observability
----------------------
If no real opponent observation arrives for `no_observation_timeout`
seconds, the node assumes the opponent is temporarily occluded/lost
and starts feeding itself pseudo-observations every
`pseudo_update_interval` seconds. Each pseudo-observation is the
median (by time-index) waypoint of the most recently generated
predicted trajectory, fed back into the sliding window exactly like a
real measurement, so the model keeps "coasting" a trajectory instead
of going stale. The moment a real observation arrives, pseudo-updates
stop immediately and the node reverts to driving off real data.

Assumptions worth double-checking against your real RacetrackUtilities
class (its source wasn't included, so these are best guesses):
  * It exposes an inverse Frenet->Cartesian method. I've called it
    `convert_to_cartesian(s, d) -> (x, y)` below. If your class names
    it something else (e.g. `frenet_to_cartesian`), update that one
    call in `predict_and_publish`.
  * `in_bounds_cartesian(x, y)` and `convert_to_frenet(x, y)` behave
    the same way for ego as they do for the opponent in the node you
    shared.
  * Ego velocity is taken as the norm of (twist.linear.x,
    twist.linear.y) rather than just linear.x, so it's robust to
    whether the odom twist is body-frame or world-frame. Swap to
    `vx` alone if your `ego_vel` training column was forward speed
    only.

No input normalization is applied, since the training script doesn't
normalize features either -- raw Frenet values are used directly on
both sides.
"""

import math
from collections import deque

import numpy as np
import rclpy
import torch
import torch.nn as nn
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from racetrack_utilities.racetrack_utilities import RacetrackUtilities

# -----------------------------
# Constants (must match training script)
# -----------------------------
HISTORY_LEN = 5
FUTURE_LEN = 10
N_FEATURES = 5  # [opp_s, opp_d, ego_s, ego_d, ego_v]


# -----------------------------
# Model definition -- copied verbatim from the training script so this
# node is self-contained. Keep this in sync with how best_model.pt was
# trained; if you change the architecture, retrain and update both.
# -----------------------------
class LSTMModel(nn.Module):
    def __init__(self, n_hidden=51):
        super(LSTMModel, self).__init__()
        self.n_hidden = n_hidden

        self.encoder_lstm1 = nn.LSTMCell(5, self.n_hidden)
        self.encoder_lstm2 = nn.LSTMCell(self.n_hidden, self.n_hidden)

        self.decoder_lstm1 = nn.LSTMCell(2, self.n_hidden)
        self.decoder_lstm2 = nn.LSTMCell(self.n_hidden, self.n_hidden)

        self.linear = nn.Linear(self.n_hidden, 2)

    def forward(self, x, future=0):
        outputs = []
        n_samples = x.size(0)

        h_t = torch.zeros(n_samples, self.n_hidden, dtype=torch.float32)
        c_t = torch.zeros(n_samples, self.n_hidden, dtype=torch.float32)
        h_t2 = torch.zeros(n_samples, self.n_hidden, dtype=torch.float32)
        c_t2 = torch.zeros(n_samples, self.n_hidden, dtype=torch.float32)

        for input_t in x.split(1, dim=1):
            input_t = input_t.squeeze(1)
            h_t, c_t = self.encoder_lstm1(input_t, (h_t, c_t))
            h_t2, c_t2 = self.encoder_lstm2(h_t, (h_t2, c_t2))
        output = self.linear(h_t2)

        for _ in range(future):
            h_t, c_t = self.decoder_lstm1(output, (h_t, c_t))
            h_t2, c_t2 = self.decoder_lstm2(h_t, (h_t2, c_t2))
            output = self.linear(h_t2)
            outputs.append(output)

        return torch.stack(outputs, dim=1)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Minimal yaw-only quaternion, avoids a tf2 dependency just for this."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class LSTMOpponentPathPredictorNode(Node):
    def __init__(self):
        super().__init__('LSTMOpponentPathPredictorNode')

        # ---- parameters ----
        self.declare_parameter(
            'map_csv_path',
            '/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv',
        )
        self.declare_parameter('model_path', 'best_model.pt')
        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('opp_frenet_topic', '/frenet_opp_state_vector')
        self.declare_parameter('predicted_path_topic', '/predicted_opponent_path')
        self.declare_parameter('no_observation_timeout', 2.0)   # seconds, "t" in the prompt
        self.declare_parameter('pseudo_update_interval', 0.25)  # seconds, 250 ms

        map_csv_path = self.get_parameter('map_csv_path').value
        model_path = self.get_parameter('model_path').value
        ego_odom_topic = self.get_parameter('ego_odom_topic').value
        opp_frenet_topic = self.get_parameter('opp_frenet_topic').value
        predicted_path_topic = self.get_parameter('predicted_path_topic').value
        self.no_observation_timeout = self.get_parameter('no_observation_timeout').value
        self.pseudo_update_interval = self.get_parameter('pseudo_update_interval').value

        # ---- racetrack + model ----
        self.racetrack = RacetrackUtilities(map_csv_path)
        meta = self.racetrack.metadata()
        self.get_logger().info(
            f"Loaded racetrack: {meta['num_points']} points, {meta['arclength']:.1f}m"
        )

        self.model = LSTMModel()
        state_dict = torch.load(model_path, map_location='cpu')
        self.model.load_state_dict(state_dict)
        self.model.eval()
        self.get_logger().info(f"Loaded LSTM weights from {model_path}")

        # ---- sliding-window state ----
        self.latest_ego_s = None
        self.latest_ego_d = None
        self.latest_ego_v = None
        self.history = deque(maxlen=HISTORY_LEN)

        # most recently generated predicted trajectory, [FUTURE_LEN, 2] of (s, d).
        # used as the source of pseudo-observations during occlusion.
        self.last_prediction = None
        # ROS time of the last *real* opponent observation
        self.last_real_obs_time = None
        # purely for logging -- tracks whether we're currently coasting on
        # pseudo-observations so we only log the mode transition once
        self.in_pseudo_mode = False

        # ---- pub/sub ----
        self.ego_odom_sub = self.create_subscription(
            Odometry, ego_odom_topic, self.ego_odom_cb, 10
        )
        self.opp_frenet_sub = self.create_subscription(
            Float64MultiArray, opp_frenet_topic, self.opp_frenet_cb, 10
        )
        self.path_pub = self.create_publisher(Path, predicted_path_topic, 10)

        # fires every `pseudo_update_interval` seconds; only takes action once
        # `no_observation_timeout` seconds have passed without a real observation
        self.pseudo_update_timer = self.create_timer(
            self.pseudo_update_interval, self.pseudo_update_cb
        )

    # -------------------------------------------------
    def ego_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.racetrack.in_bounds_cartesian(x, y):
            self.get_logger().debug(
                f"ego ({x:.2f}, {y:.2f}) outside racetrack bounds -- skipping"
            )
            return

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        ego_v = math.sqrt(vx * vx + vy * vy)

        ego_s, ego_d = self.racetrack.convert_to_frenet(x, y)

        self.latest_ego_s = ego_s
        self.latest_ego_d = ego_d
        self.latest_ego_v = ego_v

    # -------------------------------------------------
    def opp_frenet_cb(self, msg: Float64MultiArray):
        opp_s, opp_d, _dt = msg.data

        if self.in_pseudo_mode:
            self.get_logger().info(
                "Real opponent observation received -- exiting pseudo-observation "
                "mode and reverting to real-data updates"
            )
            self.in_pseudo_mode = False

        self.last_real_obs_time = self.get_clock().now()
        self._push_observation(opp_s, opp_d)

    # -------------------------------------------------
    def pseudo_update_cb(self):
        """Timer callback, fires every `pseudo_update_interval` seconds.
        Only takes action once `no_observation_timeout` seconds have
        elapsed since the last real opponent observation -- otherwise
        it's a no-op, so it's safe to just let this timer run forever."""
        if self.last_real_obs_time is None:
            return  # haven't seen a single real observation yet

        elapsed = (self.get_clock().now() - self.last_real_obs_time).nanoseconds * 1e-9
        if elapsed < self.no_observation_timeout:
            return  # still within the trusted real-observation window

        if self.last_prediction is None:
            return  # no trajectory generated yet to draw a pseudo-point from

        if not self.in_pseudo_mode:
            self.get_logger().warn(
                f"No opponent observation for {elapsed:.2f}s "
                f"(timeout={self.no_observation_timeout:.2f}s) -- switching to "
                f"pseudo-observation updates every {self.pseudo_update_interval:.2f}s"
            )
            self.in_pseudo_mode = True

        # median (by time-index) waypoint of the last predicted trajectory
        median_idx = len(self.last_prediction) // 2
        pseudo_s, pseudo_d = self.last_prediction[median_idx]
        self._push_observation(float(pseudo_s), float(pseudo_d))

    # -------------------------------------------------
    def _push_observation(self, opp_s: float, opp_d: float):
        """Shared by real and pseudo observations: combine with the latest
        ego state, push onto the sliding window, and predict once full."""
        if self.latest_ego_s is None:
            # haven't received an ego odom sample yet, nothing to combine with
            return

        feature = [
            opp_s,
            opp_d,
            self.latest_ego_s,
            self.latest_ego_d,
            self.latest_ego_v,
        ]
        self.history.append(feature)  # deque maxlen handles the sliding window

        if len(self.history) < HISTORY_LEN:
            return  # still filling the window

        self.predict_and_publish()

    # -------------------------------------------------
    def predict_and_publish(self):
        x = np.array(self.history, dtype=np.float32)        # [HISTORY_LEN, N_FEATURES]
        x_tensor = torch.from_numpy(x).unsqueeze(0)          # [1, HISTORY_LEN, N_FEATURES]

        with torch.no_grad():
            pred = self.model(x_tensor, future=FUTURE_LEN)  # [1, FUTURE_LEN, 2]

        pred_sd = pred.squeeze(0).numpy()                    # [FUTURE_LEN, 2]
        self.last_prediction = pred_sd  # cached for pseudo-observation updates

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        prev_xy = None
        for s_pred, d_pred in pred_sd:
            x_pred, y_pred = self.racetrack.convert_to_cartesian(
                float(s_pred), float(d_pred)
            )

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x_pred
            pose.pose.position.y = y_pred
            pose.pose.position.z = 0.0

            if prev_xy is not None:
                yaw = math.atan2(y_pred - prev_xy[1], x_pred - prev_xy[0])
                pose.pose.orientation = yaw_to_quaternion(yaw)
            else:
                pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            path_msg.poses.append(pose)
            prev_xy = (x_pred, y_pred)

        self.path_pub.publish(path_msg)
        self.get_logger().debug(
            f"published predicted path with {len(path_msg.poses)} poses"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LSTMOpponentPathPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()