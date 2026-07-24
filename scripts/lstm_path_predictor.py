#!/usr/bin/env python3
"""
ROS2 node that predicts and visualizes the opponent vehicle's future
trajectory using the pretrained LSTM seq2seq model from the current
training script (main.py).

This is a rewrite of the original node to match the ACTUAL trained
model architecture and feature set, which differ from what the first
version of this node assumed:

  * Features are [dt, rel_s, rel_d] (3 features) -- NOT
    [opp_s, opp_d, ego_s, ego_d, ego_v] (5 features). rel_s/rel_d are
    the opponent's position relative to ego in Frenet coordinates,
    matching TrajectoryPoint in the training script exactly. ego_v is
    not used by this model at all.
  * The model architecture is an nn.LSTM(num_layers=2) encoder feeding
    a 2-cell LSTMCell decoder that is driven at each step by an
    explicit future dt value (NOT autoregressively fed its own
    previous output). This means the caller must supply a dt schedule
    for however many future steps it wants predicted.
  * All model inputs/outputs are NORMALIZED using per-feature
    mean/std computed from the training set (see compute_norm_stats
    in main.py). These stats are saved to checkpoint/norm_stats.npz
    during training and loaded here -- if you retrain, make sure that
    file gets regenerated and this node picks up the new one.

Prediction horizon
-------------------
This node predicts PREDICTION_HORIZON_SEC (2.0s) into the future,
split into FUTURE_LEN steps of STEP_DT = horizon / FUTURE_LEN each,
matching the future_len=15 the model was trained with (the decoder is
a recurrent loop over whatever dt sequence you feed it, so it isn't
strictly locked to 15 steps -- but staying close to the training
horizon length is the safer choice for now rather than extrapolating
the decoder far outside what it was trained to unroll).

Pipeline
--------
1. Listen to ego odometry (/ego_racecar/odom). Convert ego (x, y) into
   Frenet coordinates (ego_s, ego_d) via RacetrackUtilities.
2. Listen to the opponent's absolute Frenet state
   (/frenet_opp_state_vector), published as [opp_s, opp_d, dt] by
   FrenetOpponentStateNode. Combine with the latest ego state to get
   rel_s = opp_s - ego_s (wrapped to the shorter way around the
   track) and rel_d = opp_d - ego_d.
3. Push [dt, rel_s, rel_d] into a fixed-size sliding window (deque,
   maxlen=HISTORY_LEN, matching the training history_len=30).
4. Once the window holds HISTORY_LEN samples, normalize it, run one
   forward pass with a fixed 2-second future dt schedule, and
   denormalize the predicted [rel_s, rel_d] sequence.
5. Convert each predicted (rel_s, rel_d) back to an absolute map-frame
   (x, y) -- anchored at the CURRENT ego (s, d), since we don't have a
   model of ego's own future motion -- and publish as a
   nav_msgs/Path for RViz.

Partial observability
----------------------
If no real opponent observation arrives for `no_observation_timeout`
seconds, the node assumes the opponent is temporarily occluded/lost
and starts feeding itself pseudo-observations every
`pseudo_update_interval` seconds. Each pseudo-observation is the
median (by time-index) waypoint of the most recently generated
predicted trajectory -- already in (rel_s, rel_d) space, so it can be
pushed onto the window directly, with dt set to pseudo_update_interval
-- fed back exactly like a real measurement, so the model keeps
"coasting" a trajectory instead of going stale. The moment a real
observation arrives, pseudo-updates stop immediately and the node
reverts to driving off real data.

Assumptions worth double-checking against your real RacetrackUtilities
class (its source wasn't included, so these are best guesses):
  * It exposes an inverse Frenet->Cartesian method. I've called it
    `convert_to_cartesian(s, d) -> (x, y)` below. If your class names
    it something else (e.g. `frenet_to_cartesian`), update that one
    call in `predict_and_publish`.
  * `in_bounds_cartesian(x, y)` and `convert_to_frenet(x, y)` behave
    the same way for ego as they do for the opponent in the node this
    was based on.
  * `metadata()` exposes the track's total arclength under the key
    'arclength', used to wrap rel_s to the shorter distance around a
    closed track (e.g. an opponent just ahead across the start/finish
    line should read as a small positive rel_s, not track_length minus
    a small number). If this key is named differently, update
    `wrap_rel_s` below.
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
# Constants (must match training script / checkpoint)
# -----------------------------
HISTORY_LEN = 30          # matches history_len=30 used in main.py's TrajectoryDataset calls
FUTURE_LEN = 15           # matches future_len=15 used in training
PREDICTION_HORIZON_SEC = 2.0
STEP_DT = PREDICTION_HORIZON_SEC / FUTURE_LEN   # seconds per predicted step
N_FEATURES = 3             # [dt, rel_s, rel_d] -- must match encoder_lstm's input_size


# -----------------------------
# Model definition -- copied verbatim (architecture-wise) from the
# current training script (main.py) so this node is self-contained.
# Keep this in sync with main.py's LSTMModel; if you change the
# architecture there, retrain and update this class too.
# -----------------------------
class LSTMModel(nn.Module):
    def __init__(self, n_hidden=51):
        super(LSTMModel, self).__init__()
        self.n_hidden = n_hidden

        self.encoder_lstm = nn.LSTM(
            input_size=3, hidden_size=self.n_hidden, num_layers=2,
            batch_first=True, dropout=0.2,
        )
        self.decoder_lstm1 = nn.LSTMCell(input_size=3, hidden_size=self.n_hidden)
        self.decoder_lstm2 = nn.LSTMCell(input_size=self.n_hidden, hidden_size=self.n_hidden)
        self.linear = nn.Linear(self.n_hidden, 2)

    def forward(self, x, future_dts):
        """
        x: (n, history_len, 3) normalized [dt, rel_s, rel_d]
        future_dts: (n, future_len, 1) normalized dt values to drive the decoder
        returns: (n, future_len, 2) normalized [rel_s, rel_d] predictions
        """
        n_samples = x.shape[0]

        h_e = torch.zeros(2, n_samples, self.n_hidden, dtype=torch.float32)
        c_e = torch.zeros(2, n_samples, self.n_hidden, dtype=torch.float32)

        encoder_output, (h_e, c_e) = self.encoder_lstm(x, (h_e, c_e))
        current_state = self.linear(encoder_output[:, -1, :])

        h_d1 = h_e[0]
        h_d2 = h_e[1]
        c_d1 = c_e[0]
        c_d2 = c_e[1]

        outputs = []
        future_len = future_dts.shape[1]
        for i in range(future_len):
            timesteps = future_dts[:, i, :]
            full_states = torch.cat([current_state, timesteps], dim=1)
            h_d1, c_d1 = self.decoder_lstm1(full_states, (h_d1, c_d1))
            h_d2, c_d2 = self.decoder_lstm2(h_d1, (h_d2, c_d2))
            current_state = self.linear(h_d2)
            outputs.append(current_state)

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
        self.declare_parameter('model_path', '/sim_ws/src/lidar_processing/scripts/checkpoint/best_model.pt')
        self.declare_parameter('norm_stats_path', '/sim_ws/src/lidar_processing/scripts/checkpoint/norm_stats.npz')
        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('opp_frenet_topic', '/frenet_opp_state_vector')
        self.declare_parameter('predicted_path_topic', '/predicted_opponent_path')
        self.declare_parameter('no_observation_timeout', 2.0)   # seconds
        self.declare_parameter('pseudo_update_interval', 0.25)  # seconds, 250 ms

        map_csv_path = self.get_parameter('map_csv_path').value
        model_path = self.get_parameter('model_path').value
        norm_stats_path = self.get_parameter('norm_stats_path').value
        ego_odom_topic = self.get_parameter('ego_odom_topic').value
        opp_frenet_topic = self.get_parameter('opp_frenet_topic').value
        predicted_path_topic = self.get_parameter('predicted_path_topic').value
        self.no_observation_timeout = self.get_parameter('no_observation_timeout').value
        self.pseudo_update_interval = self.get_parameter('pseudo_update_interval').value

        # ---- racetrack ----
        self.racetrack = RacetrackUtilities(map_csv_path)
        meta = self.racetrack.metadata()
        self.track_length = meta['arclength']
        self.get_logger().info(
            f"Loaded racetrack: {meta['num_points']} points, {self.track_length:.1f}m"
        )

        # ---- normalization stats ----
        # These MUST come from the same training run that produced model_path.
        # Loading mismatched stats will silently produce wrong predictions
        # (no error -- the shapes all still line up, the numbers are just
        # scaled incorrectly).
        stats = np.load(norm_stats_path)
        self.mean = stats['mean'].astype(np.float32)   # [dt, rel_s, rel_d]
        self.std = stats['std'].astype(np.float32)
        self.get_logger().info(
            f"Loaded normalization stats from {norm_stats_path}: "
            f"mean={self.mean}, std={self.std}"
        )

        # ---- model ----
        self.model = LSTMModel()
        state_dict = torch.load(model_path, map_location='cpu')
        self.model.load_state_dict(state_dict)
        self.model.eval()
        self.get_logger().info(f"Loaded LSTM weights from {model_path}")

        # Precompute the fixed future dt schedule for a PREDICTION_HORIZON_SEC
        # lookahead, normalized using the dt column's mean/std. This is the
        # same tensor every call, so build it once.
        raw_future_dts = np.full((1, FUTURE_LEN, 1), STEP_DT, dtype=np.float32)
        norm_future_dts = (raw_future_dts - self.mean[0]) / self.std[0]
        self.future_dts_tensor = torch.from_numpy(norm_future_dts)

        # ---- sliding-window state ----
        self.latest_ego_s = None
        self.latest_ego_d = None
        self.history = deque(maxlen=HISTORY_LEN)  # each entry: [dt, rel_s, rel_d]

        # most recently generated predicted trajectory, [FUTURE_LEN, 2] of
        # (rel_s, rel_d) -- used as the source of pseudo-observations during
        # occlusion.
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
    def wrap_rel_s(self, rel_s: float) -> float:
        """Wrap a raw s-difference to the shorter way around a closed
        track, e.g. an opponent just ahead across the start/finish line
        should read as a small positive rel_s, not track_length minus a
        small number."""
        half = self.track_length / 2.0
        return ((rel_s + half) % self.track_length) - half

    # -------------------------------------------------
    def ego_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.racetrack.in_bounds_cartesian(x, y):
            self.get_logger().debug(
                f"ego ({x:.2f}, {y:.2f}) outside racetrack bounds -- skipping"
            )
            return

        ego_s, ego_d = self.racetrack.convert_to_frenet(x, y)

        self.latest_ego_s = ego_s
        self.latest_ego_d = ego_d

    # -------------------------------------------------
    def opp_frenet_cb(self, msg: Float64MultiArray):
        opp_s, opp_d, dt = msg.data

        if self.in_pseudo_mode:
            self.get_logger().info(
                "Real opponent observation received -- exiting pseudo-observation "
                "mode and reverting to real-data updates"
            )
            self.in_pseudo_mode = False

        self.last_real_obs_time = self.get_clock().now()

        if self.latest_ego_s is None:
            return  # haven't received an ego odom sample yet

        rel_s = self.wrap_rel_s(opp_s - self.latest_ego_s)
        rel_d = opp_d - self.latest_ego_d
        self._push_observation(dt, rel_s, rel_d)

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

        # median (by time-index) waypoint of the last predicted trajectory.
        # Already in (rel_s, rel_d) space, so it can be pushed directly.
        median_idx = len(self.last_prediction) // 2
        pseudo_rel_s, pseudo_rel_d = self.last_prediction[median_idx]
        self._push_observation(self.pseudo_update_interval, float(pseudo_rel_s), float(pseudo_rel_d))

    # -------------------------------------------------
    def _push_observation(self, dt: float, rel_s: float, rel_d: float):
        """Shared by real and pseudo observations."""
        self.history.append([dt, rel_s, rel_d])  # deque maxlen handles the sliding window

        if len(self.history) < HISTORY_LEN:
            return  # still filling the window

        self.predict_and_publish()

    # -------------------------------------------------
    def predict_and_publish(self):
        x_raw = np.array(self.history, dtype=np.float32)       # [HISTORY_LEN, 3]
        x_norm = (x_raw - self.mean) / self.std
        x_tensor = torch.from_numpy(x_norm).unsqueeze(0)       # [1, HISTORY_LEN, 3]

        with torch.no_grad():
            pred_norm = self.model(x_tensor, self.future_dts_tensor)  # [1, FUTURE_LEN, 2]

        # denormalize predicted [rel_s, rel_d] using the position columns'
        # mean/std (indices 1, 2 -- index 0 is dt)
        pred_norm = pred_norm.squeeze(0).numpy()                # [FUTURE_LEN, 2]
        pred_rel = pred_norm * self.std[1:] + self.mean[1:]     # [FUTURE_LEN, 2], raw units

        self.last_prediction = pred_rel  # cached for pseudo-observation updates

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Anchor predicted relative positions to the CURRENT ego (s, d).
        # We don't have a model of ego's own future motion, so this is an
        # approximation: it assumes ego holds its current Frenet position
        # for the purpose of converting the opponent's predicted relative
        # offsets back into absolute map-frame coordinates. Good enough for
        # visualization over a short (2s) horizon; revisit if ego moves
        # fast relative to the horizon length.
        anchor_s = self.latest_ego_s
        anchor_d = self.latest_ego_d

        prev_xy = None
        for rel_s_pred, rel_d_pred in pred_rel:
            abs_s = self.wrap_rel_s(anchor_s + rel_s_pred) if self.track_length else anchor_s + rel_s_pred
            abs_s = abs_s % self.track_length
            abs_d = anchor_d + rel_d_pred

            x_pred, y_pred = self.racetrack.convert_to_cartesian(float(abs_s), float(abs_d))

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
            f"published {PREDICTION_HORIZON_SEC:.1f}s predicted path "
            f"with {len(path_msg.poses)} poses"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LSTMOpponentPathPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()