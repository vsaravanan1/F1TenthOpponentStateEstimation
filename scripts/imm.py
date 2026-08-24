#!/usr/bin/env python3
"""
IMM (Interacting Multiple Model) Filter Node for Frenet-frame trajectory prediction.

Subscribes to /predicted_opponent_path  → nav_msgs/Path (LSTM node, map frame)
Publishes    to /imm_path               → nav_msgs/Path in "map" frame

The LSTM node already handles partial observability internally (pseudo-
observations during occlusion).  This node therefore only needs one input:
the LSTM's predicted path.  On every incoming path message it extracts the
FIRST pose only (the model's current-state estimate, closest in time to
now), converts it back to Frenet coordinates, and feeds it into the IMM as
a measurement with the regular R matrix.  Future waypoints are ignored.

dt for the IMM update is computed from the wall-clock inter-arrival time of
consecutive path messages.  A large gap (> REINIT_THRESHOLD) triggers a
cold re-initialisation of all filters.

Forward-propagates N_STEPS × PREDICT_DT seconds ahead using the
mode-probability-weighted combined F matrix and publishes the result.
"""

from typing import List, Optional, Tuple

import numpy as np
import rclpy
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from racetrack_utilities.racetrack_utilities import RacetrackUtilities


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
PREDICT_DT       = 0.1   # seconds per IMM forward-propagation step
N_STEPS          = 10    # number of steps to forward-propagate
REINIT_THRESHOLD = 2.0   # seconds; re-init all filters if gap is this large


# ──────────────────────────────────────────────────────────────────────────────
# Wrapped Kalman Filter (cyclic / modular innovation for s)
# ──────────────────────────────────────────────────────────────────────────────
class WrappedKalmanFilter(KalmanFilter):
    """
    Kalman filter whose innovation is computed with modular (cyclic) arithmetic.
    Useful when the observed quantity wraps around (e.g. arc-length on a closed
    loop).  Set wrap_length=None to fall back to a standard KF.
    """

    def __init__(self, dim_x: int, dim_z: int, wrap_length: Optional[float] = None):
        super().__init__(dim_x=dim_x, dim_z=dim_z)
        self.wrap_length = wrap_length

    def _wrapped_residual(self, z: np.ndarray) -> np.ndarray:
        y = z - self.H @ self.x
        if self.wrap_length is not None:
            half = self.wrap_length / 2.0
            y = (y + half) % self.wrap_length - half
        return y

    def update(self, z, R=None, H=None):
        if z is None:
            self.z      = np.array([[None] * self.dim_z]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            self.y      = np.zeros((self.dim_z, 1))
            return

        from filterpy.common import reshape_z
        z = reshape_z(z, self.dim_z, self.x.ndim)

        if R is None:
            R = self.R
        if H is None:
            H = self.H

        self._log_likelihood = None
        self._likelihood     = None
        self._mahalanobis    = None

        self.y  = self._wrapped_residual(z)
        PHT     = self.P @ H.T
        self.S  = H @ PHT + R
        self.SI = np.linalg.inv(self.S)
        self.K  = PHT @ self.SI

        self.x = self.x + self.K @ self.y
        I_KH   = self._I - self.K @ H
        self.P = I_KH @ self.P @ I_KH.T + self.K @ R @ self.K.T

        self.z      = z.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()


# ──────────────────────────────────────────────────────────────────────────────
# Filter factories
# ──────────────────────────────────────────────────────────────────────────────
def make_s_cv_filter(dt: float, wrap_length: Optional[float] = None) -> WrappedKalmanFilter:
    """Constant longitudinal velocity.  State: [s, s', s'']  Observe: [s]"""
    f = WrappedKalmanFilter(dim_x=3, dim_z=1, wrap_length=wrap_length)
    f.F = np.array([[1, dt, 0],
                    [0,  1, 0],
                    [0,  0, 0]], dtype=float)
    f.H = np.array([[1, 0, 0]], dtype=float)
    f.P = np.eye(3)
    f.R = np.eye(1) * 0.3
    f.Q = np.eye(1) * 0.5
    return f


def make_s_ca_filter(dt: float, wrap_length: Optional[float] = None) -> WrappedKalmanFilter:
    """Constant longitudinal acceleration.  State: [s, s', s'']  Observe: [s]"""
    f = WrappedKalmanFilter(dim_x=3, dim_z=1, wrap_length=wrap_length)
    f.F = np.array([[1, dt, 0.5 * dt**2],
                    [0,  1,          dt],
                    [0,  0,           1]], dtype=float)
    f.H = np.array([[1, 0, 0]], dtype=float)
    f.P = np.eye(3)
    f.R = np.eye(1) * 0.3
    f.Q = np.eye(3)
    return f


def make_d_cd_filter(dt: float) -> KalmanFilter:
    """Constant lateral deviation.  State: [d, d']  Observe: [d]"""
    f = KalmanFilter(dim_x=2, dim_z=1)
    f.F = np.array([[1, 0],
                    [0, 0]], dtype=float)
    f.H = np.array([[1, 0]], dtype=float)
    f.P = np.eye(2)
    f.R = np.eye(1) * 0.3
    f.Q = np.eye(2)
    return f


def make_d_cv_filter(dt: float) -> KalmanFilter:
    """Constant lateral velocity.  State: [d, d']  Observe: [d]"""
    f = KalmanFilter(dim_x=2, dim_z=1)
    f.F = np.array([[1, dt],
                    [0,  1]], dtype=float)
    f.H = np.array([[1, 0]], dtype=float)
    f.P = np.eye(2)
    f.R = np.eye(1) * 0.3
    f.Q = np.eye(2)
    return f


# ──────────────────────────────────────────────────────────────────────────────
# F-matrix updaters (in-place, called before each predict step)
# ──────────────────────────────────────────────────────────────────────────────
def _update_s_F(kf: KalmanFilter, dt: float) -> None:
    """Rewrite F for an s-axis filter based on whether it is CV or CA."""
    if kf.F[0, 2] == 0 and kf.F[1, 2] == 0:  # CV: third column zeroed
        kf.F = np.array([[1, dt, 0],
                         [0,  1, 0],
                         [0,  0, 0]], dtype=float)
    else:                                        # CA
        kf.F = np.array([[1, dt, 0.5 * dt**2],
                         [0,  1,          dt],
                         [0,  0,           1]], dtype=float)


def _update_d_F(kf: KalmanFilter, dt: float) -> None:
    """Rewrite F for a d-axis filter based on whether it is CD or CV."""
    if kf.F[0, 1] != 0:   # CV: off-diagonal term present
        kf.F = np.array([[1, dt],
                         [0,  1]], dtype=float)
    # CD model: F = [[1,0],[0,0]] — unchanged regardless of dt


# ──────────────────────────────────────────────────────────────────────────────
# IMMAxis — all state for one Frenet axis
# ──────────────────────────────────────────────────────────────────────────────
class IMMAxis:
    """Holds and operates on one axis (s or d) of the IMM."""

    def __init__(
        self,
        filters: List[KalmanFilter],
        mu0: np.ndarray,
        M: np.ndarray,
        axis: str,
    ):
        self.filters = filters
        self.mu      = mu0.copy()
        self.M       = M
        self.axis    = axis
        self.xs      = [f.x.copy() for f in filters]
        self.Ps      = [f.P.copy() for f in filters]

    # ------------------------------------------------------------------
    def update(self, z: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        One full IMM cycle: mix → predict → update → reweight.

        Args:
            z:  measurement column vector, e.g. np.array([[s_value]])
            dt: elapsed time since last update (used to refresh F matrices)

        Returns:
            (x_combined, P_combined) — the mode-probability-weighted estimate
        """
        n = len(self.filters)

        # 1. Update F matrices for this dt
        for kf in self.filters:
            (_update_s_F if self.axis == 's' else _update_d_F)(kf, dt)

        # 2. Mixing probabilities
        c_bar = self.M.T @ self.mu                          # (n,)
        mu_ij = (self.M * self.mu[:, None]) / c_bar[None, :]  # (n, n)

        # 3. Interaction / mixing
        xs_mix, Ps_mix = [], []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.xs[i] for i in range(n))
            P_j = sum(
                mu_ij[i, j] * (
                    self.Ps[i] + np.outer(self.xs[i] - x_j, self.xs[i] - x_j)
                )
                for i in range(n)
            )
            xs_mix.append(x_j)
            Ps_mix.append(P_j)

        # 4. Mode-conditioned predict + update; compute likelihoods
        likelihoods = np.zeros(n)
        for j, kf in enumerate(self.filters):
            kf.x = xs_mix[j].copy()
            kf.P = Ps_mix[j].copy()
            kf.predict()
            kf.update(z)
            self.xs[j] = kf.x.copy()
            self.Ps[j] = kf.P.copy()
            try:
                y, S = kf.y, kf.S
                sign, logdet = np.linalg.slogdet(S)
                maha = float(y.T @ np.linalg.inv(S) @ y)
                likelihoods[j] = np.exp(
                    -0.5 * (maha + logdet + y.shape[0] * np.log(2 * np.pi))
                )
            except Exception:
                likelihoods[j] = 1e-300

        # 5. Update mode probabilities
        raw   = likelihoods * c_bar
        total = raw.sum()
        self.mu = raw / total if total > 1e-300 else np.ones(n) / n

        # 6. Combined estimate
        x = sum(self.mu[j] * self.xs[j] for j in range(n))
        P = sum(
            self.mu[j] * (
                self.Ps[j] + np.outer(self.xs[j] - x, self.xs[j] - x)
            )
            for j in range(n)
        )
        return x, P

    # ------------------------------------------------------------------
    def weighted_F(self) -> np.ndarray:
        """Mode-probability-weighted average of constituent F matrices."""
        return sum(self.mu[j] * self.filters[j].F for j in range(len(self.filters)))

    # ------------------------------------------------------------------
    def reinitialize(self, x0: np.ndarray) -> None:
        """Hard-reset all filters to x0 with identity covariance."""
        n = len(self.filters)
        for j, kf in enumerate(self.filters):
            kf.x       = x0.copy()
            kf.P       = np.eye(kf.dim_x)
            self.xs[j] = x0.copy()
            self.Ps[j] = np.eye(kf.dim_x)
        self.mu = np.ones(n) / n


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────────────────────────────────────
class IMMFilterNode(Node):

    def __init__(self):
        super().__init__('imm_filter_node')

        map_csv = '/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv'
        self.racetrack_utilities = RacetrackUtilities(map_csv)

        # ── s-axis IMM ────────────────────────────────────────────────────────
        dt0        = PREDICT_DT
        wrap_len   = self.racetrack_utilities.arclength
        s_filters  = [
            make_s_cv_filter(dt0, wrap_length=wrap_len),  # model 0: CV
            make_s_ca_filter(dt0, wrap_length=wrap_len),  # model 1: CA
        ]
        self.s_imm = IMMAxis(
            s_filters,
            mu0=np.array([0.5, 0.5]),
            M=np.array([[0.95, 0.05],
                        [0.10, 0.90]]),
            axis='s',
        )

        # ── d-axis IMM ────────────────────────────────────────────────────────
        d_filters  = [
            make_d_cd_filter(dt0),   # model 0: constant d
            make_d_cv_filter(dt0),   # model 1: constant d-velocity
        ]
        self.d_imm = IMMAxis(
            d_filters,
            mu0=np.array([0.5, 0.5]),
            M=np.array([[0.9, 0.1],
                        [0.1, 0.9]]),
            axis='d',
        )

        # ── node state ────────────────────────────────────────────────────────
        self._initialized         = False
        self._last_path_ros_time  = None   # rclpy.time.Time of last LSTM path msg

        # ── ROS2 I/O ─────────────────────────────────────────────────────────
        # self.sub = self.create_subscription(
        #     Path,
        #     '/predicted_opponent_path',
        #     self._lstm_path_cb,
        #     10,
        # )
        self.pub = self.create_publisher(Path, '/imm_path', 10)

        self.get_logger().info('IMMFilterNode ready — listening to LSTM path.')

    # ──────────────────────────────────────────────────────────────────────────
    def _lstm_path_cb(self, msg: Path) -> None:
        """
        Fires whenever the LSTM node publishes a new predicted path.

        Only the FIRST pose in the path is used — that is the model's current-
        state estimate (the position closest in time to now).  All future
        waypoints in the path are intentionally ignored here; the IMM's own
        forward propagation handles future prediction.
        """
        if not msg.poses:
            self.get_logger().debug('Received empty LSTM path — skipping.')
            return

        # ── Extract the current-state estimate (first pose only) ──────────────
        first_pose = msg.poses[0]
        x_cart = first_pose.pose.position.x
        y_cart = first_pose.pose.position.y

        if not self.racetrack_utilities.in_bounds_cartesian(x_cart, y_cart):
            self.get_logger().debug(
                f'First LSTM pose ({x_cart:.2f}, {y_cart:.2f}) '
                f'is outside racetrack bounds — skipping.'
            )
            return

        s, d = self.racetrack_utilities.convert_to_frenet(x_cart, y_cart)
        s, d = float(s), float(d)

        # ── Compute dt from inter-arrival time ────────────────────────────────
        now = self.get_clock().now()
        if self._last_path_ros_time is None:
            dt = 0.0
        else:
            dt = (now - self._last_path_ros_time).nanoseconds * 1e-9
        self._last_path_ros_time = now

        # ── Cold start ────────────────────────────────────────────────────────
        if not self._initialized:
            self.s_imm.reinitialize(np.array([s, 0.0, 0.0]))
            self.d_imm.reinitialize(np.array([d, 0.0]))
            self._initialized = True
            self.get_logger().info(
                f'IMM initialised from LSTM path — s={s:.3f}, d={d:.3f}'
            )
            return

        # ── Large gap: re-initialise to prevent filter divergence ─────────────
        if dt > REINIT_THRESHOLD:
            self.get_logger().warn(
                f'Large dt={dt:.2f}s between LSTM path messages — '
                f'reinitialising IMM filters.'
            )
            self.s_imm.reinitialize(np.array([s, 0.0, 0.0]))
            self.d_imm.reinitialize(np.array([d, 0.0]))
            return

        # ── Normal IMM update with regular R ─────────────────────────────────
        z_s = np.array([[s]])
        z_d = np.array([[d]])

        x_s, _ = self.s_imm.update(z_s, dt)
        x_d, _ = self.d_imm.update(z_d, dt)

        # ── Forward-propagate and publish ─────────────────────────────────────
        predicted = self._forward_propagate(x_s, x_d)
        self._publish_path(predicted)

    # ──────────────────────────────────────────────────────────────────────────
    def _forward_propagate(
        self, x_s: np.ndarray, x_d: np.ndarray
    ) -> List[Tuple[float, float]]:
        """Propagate N_STEPS ahead using the IMM-weighted combined F matrix."""
        F_s     = self.s_imm.weighted_F()
        F_d     = self.d_imm.weighted_F()
        state_s = x_s.copy()
        state_d = x_d.copy()

        predicted = []
        for _ in range(N_STEPS):
            state_s = F_s @ state_s
            state_d = F_d @ state_d
            predicted.append((float(state_s[0]), float(state_d[0])))

        return predicted

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_path(self, predicted: List[Tuple[float, float]]) -> None:
        """Convert (s, d) pairs to Cartesian and publish as nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for ps, pd in predicted:
            if ps > self.racetrack_utilities.arclength:
                ps -= self.racetrack_utilities.arclength
            x_cart, y_cart = self.racetrack_utilities.convert_to_cartesian(ps, pd)

            pose = PoseStamped()
            pose.header              = path_msg.header
            pose.pose.position.x     = x_cart
            pose.pose.position.y     = y_cart
            pose.pose.position.z     = 0.0
            path_msg.poses.append(pose)

        self.pub.publish(path_msg)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = IMMFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()