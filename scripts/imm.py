#!/usr/bin/env python3
"""
IMM (Interacting Multiple Model) Filter Node for Frenet-frame trajectory prediction.

Subscribes to /frenet_opp_state_vector  → (s, d, dt)
Publishes    to /imm_path               → nav_msgs/Path in "map" frame

Forward-propagates 10 timesteps × 0.25 s = 2.5 s ahead.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from racetrack_utilities.racetrack_utilities import RacetrackUtilities

from filterpy.kalman import KalmanFilter

from typing import Optional, List, Tuple


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
PREDICT_DT = 0.1          # seconds per forward-prop step
N_STEPS    = 10            # number of steps to forward-propagate
REINIT_THRESHOLD = 2.0     # seconds; reinitialize KF if dt gap is too large


# ──────────────────────────────────────────────────────────────────────────────
# Wrapped Kalman Filter (cyclic / modular innovation for s)
# ──────────────────────────────────────────────────────────────────────────────
class WrappedKalmanFilter(KalmanFilter):
    """
    Kalman filter whose innovation is computed with modular (cyclic) arithmetic.
    Useful when the observed quantity wraps around (e.g. arc-length on a closed
    loop). For open tracks set wrap_length=None to fall back to standard KF.
    """

    def __init__(self, dim_x: int, dim_z: int, wrap_length: Optional[float] = None):
        super().__init__(dim_x=dim_x, dim_z=dim_z)
        self.wrap_length = wrap_length  # e.g. track circumference in metres

    def _wrapped_residual(self, z):
        """Return (z - Hx) with cyclic correction."""
        y = z - self.H @ self.x
        if self.wrap_length is not None:
            half = self.wrap_length / 2.0
            y = (y + half) % self.wrap_length - half
        return y

    def update(self, z, R=None, H=None):
        """Override update to inject the wrapped (cyclic) innovation."""
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

        # Reset cached properties (filterpy uses lazy @property pattern)
        self._log_likelihood = None
        self._likelihood     = None
        self._mahalanobis    = None

        # Innovation with cyclic correction
        self.y = self._wrapped_residual(z)          # cyclic residual

        PHT     = self.P @ H.T
        self.S  = H @ PHT + R
        self.SI = np.linalg.inv(self.S)
        self.K  = PHT @ self.SI

        self.x = self.x + self.K @ self.y
        I_KH   = self._I - self.K @ H
        self.P = I_KH @ self.P @ I_KH.T + self.K @ R @ self.K.T   # Joseph form

        self.z      = z.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()


# ──────────────────────────────────────────────────────────────────────────────
# Filter factories
# ──────────────────────────────────────────────────────────────────────────────
def make_s_cv_filter(dt: float, wrap_length: Optional[float] = None) -> WrappedKalmanFilter:
    """
    Constant longitudinal velocity model.
    State  : [s, s', s'']
    Observe: [s]
    """
    f = WrappedKalmanFilter(dim_x=3, dim_z=1, wrap_length=wrap_length)
    f.F = np.array([[1, dt, 0],
                    [0,  1, 0],
                    [0,  0, 0]], dtype=float)
    f.H = np.array([[1, 0, 0]], dtype=float)
    f.P = np.eye(3)
    f.R = np.eye(1) * 0.3
    f.Q = np.eye(1) * 0.5   # scaled for CV model
    return f


def make_s_ca_filter(dt: float, wrap_length: Optional[float] = None) -> WrappedKalmanFilter:
    """
    Constant longitudinal acceleration model.
    State  : [s, s', s'']
    Observe: [s]
    """
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
    """
    Constant deviation (constant d) model.
    State  : [d, d']
    Observe: [d]
    """
    f = KalmanFilter(dim_x=2, dim_z=1)
    f.F = np.array([[1, 0],
                    [0, 0]], dtype=float)
    f.H = np.array([[1, 0]], dtype=float)
    f.P = np.eye(2)
    f.R = np.eye(1) * 0.3
    f.Q = np.eye(2)
    return f


def make_d_cv_filter(dt: float) -> KalmanFilter:
    """
    Constant lateral velocity model.
    State  : [d, d']
    Observe: [d]
    """
    f = KalmanFilter(dim_x=2, dim_z=1)
    f.F = np.array([[1, dt],
                    [0,  1]], dtype=float)
    f.H = np.array([[1, 0]], dtype=float)
    f.P = np.eye(2)
    f.R = np.eye(1) * 0.3
    f.Q = np.eye(2)
    return f


# ──────────────────────────────────────────────────────────────────────────────
# IMM helper: predict with dt-updated F matrices
# ──────────────────────────────────────────────────────────────────────────────
def _update_s_F(kf: KalmanFilter, dt: float) -> None:
    """Rewrite F in-place for s-filters based on current dt."""
    if kf.F[0, 2] == 0 and kf.F[1, 2] == 0:
        # CV model (third row/col zeroed out)
        kf.F = np.array([[1, dt, 0],
                         [0,  1, 0],
                         [0,  0, 0]], dtype=float)
    else:
        # CA model
        kf.F = np.array([[1, dt, 0.5 * dt**2],
                         [0,  1,          dt],
                         [0,  0,           1]], dtype=float)


def _update_d_F(kf: KalmanFilter, dt: float) -> None:
    """Rewrite F in-place for d-filters based on current dt."""
    if kf.F[0, 1] == 0:
        # Constant-d model — F stays [[1,0],[0,0]]
        pass
    else:
        # CV model
        kf.F = np.array([[1, dt],
                         [0,  1]], dtype=float)


def predict_imm_with_dt(
        filters,            # List[KalmanFilter]
        mu,                 # np.ndarray
        M,                  # np.ndarray
        xs,                 # List[np.ndarray]
        Ps,                 # List[np.ndarray]
        dt,                 # float
        axis='s'            # str
):
    # type: (...) -> Tuple[np.ndarray, np.ndarray, list, list, np.ndarray]
    """
    One IMM prediction step with dt-updated transition matrices.

    Parameters
    ----------
    filters : constituent KF list
    mu      : current mode probabilities  (n_models,)
    M       : Markov transition matrix    (n_models, n_models)
    xs      : list of per-filter state vectors
    Ps      : list of per-filter covariance matrices
    dt      : elapsed time
    axis    : 's' or 'd' — selects which F-updater to use

    Returns
    -------
    x_combined : mixed/combined state
    P_combined : mixed/combined covariance
    xs_new     : updated per-filter states (after predict)
    Ps_new     : updated per-filter covariances (after predict)
    mu_pred    : predicted mode probabilities
    """
    n = len(filters)

    # 1. Update F matrices for this dt
    for kf in filters:
        if axis == 's':
            _update_s_F(kf, dt)
        else:
            _update_d_F(kf, dt)

    # 2. Mixing probabilities
    c_bar  = M.T @ mu                              # (n,)
    mu_ij  = (M * mu[:, None]) / c_bar[None, :]   # (n, n)

    # 3. Mixing (interaction)
    xs_mix = []
    Ps_mix = []
    for j in range(n):
        x_j = sum(mu_ij[i, j] * xs[i] for i in range(n))
        P_j = sum(mu_ij[i, j] * (Ps[i] + np.outer(xs[i] - x_j, xs[i] - x_j))
                  for i in range(n))
        xs_mix.append(x_j)
        Ps_mix.append(P_j)

    # 4. Mode-conditioned predict
    xs_new = []
    Ps_new = []
    for j, kf in enumerate(filters):
        kf.x = xs_mix[j].copy()
        kf.P = Ps_mix[j].copy()
        kf.predict()
        xs_new.append(kf.x.copy())
        Ps_new.append(kf.P.copy())

    # 5. Combined prediction (for output)
    x_combined = sum(mu[j] * xs_new[j] for j in range(n))
    P_combined = sum(mu[j] * (Ps_new[j] + np.outer(xs_new[j] - x_combined,
                                                     xs_new[j] - x_combined))
                     for j in range(n))

    return x_combined, P_combined, xs_new, Ps_new, c_bar


# ──────────────────────────────────────────────────────────────────────────────
# Lightweight IMM state container
# ──────────────────────────────────────────────────────────────────────────────
class IMMAxis:
    """Holds all state for one IMM axis (s or d)."""

    def __init__(self,
                 filters,       # List[KalmanFilter]
                 mu0,           # np.ndarray
                 M,             # np.ndarray
                 axis           # str
                 ):
        self.filters = filters
        self.mu      = mu0.copy()
        self.M       = M
        self.axis    = axis
        self.xs      = [f.x.copy() for f in filters]
        self.Ps      = [f.P.copy() for f in filters]

    def predict(self, dt):
        # type: (float) -> Tuple[np.ndarray, np.ndarray]
        x, P, self.xs, self.Ps, self.mu = predict_imm_with_dt(
            self.filters, self.mu, self.M, self.xs, self.Ps, dt, self.axis)
        return x, P

    def update(self, z, dt):
        # type: (np.ndarray, float) -> Tuple[np.ndarray, np.ndarray]
        """Predict then update all filters, recompute mode probabilities."""
        n = len(self.filters)

        # Update F and mix
        for kf in self.filters:
            if self.axis == 's':
                _update_s_F(kf, dt)
            else:
                _update_d_F(kf, dt)

        c_bar = self.M.T @ self.mu
        mu_ij = (self.M * self.mu[:, None]) / c_bar[None, :]

        # Mix
        xs_mix, Ps_mix = [], []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.xs[i] for i in range(n))
            P_j = sum(mu_ij[i, j] * (self.Ps[i] +
                       np.outer(self.xs[i] - x_j, self.xs[i] - x_j))
                      for i in range(n))
            xs_mix.append(x_j)
            Ps_mix.append(P_j)

        # Predict + update each filter
        likelihoods = np.zeros(n)
        for j, kf in enumerate(self.filters):
            kf.x = xs_mix[j].copy()
            kf.P = Ps_mix[j].copy()
            kf.predict()
            kf.update(z)
            self.xs[j] = kf.x.copy()
            self.Ps[j] = kf.P.copy()
            # Likelihood from innovation
            y = kf.y
            S = kf.S
            try:
                sign, logdet = np.linalg.slogdet(S)
                maha = float(y.T @ np.linalg.inv(S) @ y)
                likelihoods[j] = np.exp(-0.5 * (maha + logdet +
                                                  y.shape[0] * np.log(2 * np.pi)))
            except Exception:
                likelihoods[j] = 1e-300

        # Update mode probabilities
        raw   = likelihoods * c_bar
        total = raw.sum()
        self.mu = raw / total if total > 1e-300 else np.ones(n) / n

        # Combined estimate
        x = sum(self.mu[j] * self.xs[j] for j in range(n))
        P = sum(self.mu[j] * (self.Ps[j] +
                 np.outer(self.xs[j] - x, self.xs[j] - x))
                for j in range(n))
        return x, P

    def weighted_F(self):
        # type: () -> np.ndarray
        """Return mode-probability-weighted average of constituent F matrices."""
        return sum(self.mu[j] * self.filters[j].F for j in range(len(self.filters)))

    def reinitialize(self, x0):
        # type: (np.ndarray) -> None
        """Hard-reset all filters to x0 (large dt gap recovery)."""
        for j, kf in enumerate(self.filters):
            kf.x       = x0.copy()
            kf.P       = np.eye(kf.dim_x)
            self.xs[j] = x0.copy()
            self.Ps[j] = np.eye(kf.dim_x)
        n       = len(self.filters)
        self.mu = np.ones(n) / n


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────────────────────────────────────
class IMMFilterNode(Node):

    def __init__(self):
        super().__init__('imm_filter_node')

        name = "/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv"
        self.racetrack_utilities = RacetrackUtilities(name)

        # build s-axis IMM
        dt0 = PREDICT_DT
        s_filters = [
            make_s_cv_filter(dt0, wrap_length=self.racetrack_utilities.arclength),   # model 0: CV
            make_s_ca_filter(dt0, wrap_length=self.racetrack_utilities.arclength),   # model 1: CA
        ]
        s_mu0 = np.array([0.5, 0.5])
        s_M   = np.array([[0.95, 0.05],
                           [0.1, 0.9]])
        self.s_imm = IMMAxis(s_filters, s_mu0, s_M, axis='s')

        # build d-axis IMM
        d_filters = [
            make_d_cd_filter(dt0),   # model 0: constant d
            make_d_cv_filter(dt0),   # model 1: constant d-velocity
        ]
        d_mu0 = np.array([0.5, 0.5])
        d_M   = np.array([[0.9, 0.1],
                           [0.1, 0.9]])
        self.d_imm = IMMAxis(d_filters, d_mu0, d_M, axis='d')

        self._initialized = False
        self._last_s      = None
        self._last_d      = None

        # ROS2 I/O
        self.sub = self.create_subscription(
            Float64MultiArray, '/frenet_opp_state_vector', self._cb, 10)
        self.pub = self.create_publisher(Path, '/imm_path', 10)

        self.get_logger().info('IMMFilterNode ready.')

    def _cb(self, msg):
        print(self.racetrack_utilities.arclength)
        # type: (Float64MultiArray) -> None
        s, d, dt = msg.data[0], msg.data[1], msg.data[2]
        z_s = np.array([[s]])
        z_d = np.array([[d]])

        # first observation: cold-start
        if not self._initialized:
            x0_s = np.array([s, 0.0, 0.0])
            x0_d = np.array([d, 0.0])
            self.s_imm.reinitialize(x0_s)
            self.d_imm.reinitialize(x0_d)
            self._initialized = True
            self._last_s = s
            self._last_d = d
            self.get_logger().info('IMM initialised — s={:.3f}, d={:.3f}'.format(s, d))
            return

        # large gap: reinitialise to prevent divergence
        if dt > REINIT_THRESHOLD:
            self.get_logger().warn(
                'Large dt={:.2f}s — reinitialising IMM filters.'.format(dt))
            x0_s = np.array([s, 0.0, 0.0])
            x0_d = np.array([d, 0.0])
            self.s_imm.reinitialize(x0_s)
            self.d_imm.reinitialize(x0_d)
            self._last_s = s
            self._last_d = d
            return

        # normal update
        x_s, _ = self.s_imm.update(z_s, dt)
        x_d, _ = self.d_imm.update(z_d, dt)

        self._last_s = x_s[0]
        self._last_d = x_d[0]

        # forward propagation
        predicted = self._forward_propagate(x_s, x_d)

        # print predictions
        print('\n── IMM forward-propagated path ──')
        print('  {:>4}  {:>6}  {:>10}  {:>10}'.format('step', 't (s)', 's', 'd'))
        for k, (ps, pd) in enumerate(predicted):
            t = (k + 1) * PREDICT_DT
            print('  {:>4}  {:>6.2f}  {:>10.4f}  {:>10.4f}'.format(k + 1, t, ps, pd))

        # publish /imm_path
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for ps, pd in predicted:
            if ps > self.racetrack_utilities.arclength:
                ps -= self.racetrack_utilities.arclength
            x_cart, y_cart = self.racetrack_utilities.convert_to_cartesian(ps, pd)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x_cart
            pose.pose.position.y = y_cart
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.pub.publish(path_msg)

    def _forward_propagate(self, x_s, x_d):
        # type: (np.ndarray, np.ndarray) -> List[Tuple[float, float]]
        """
        Propagate N_STEPS ahead using the IMM-weighted combined F matrix.
        Operates on copies so the live IMM state is not mutated.
        """
        F_s = self.s_imm.weighted_F()   # 3×3
        F_d = self.d_imm.weighted_F()   # 2×2

        state_s = x_s.copy()
        state_d = x_d.copy()

        predicted = []
        for _ in range(N_STEPS):
            state_s = F_s @ state_s
            state_d = F_d @ state_d
            predicted.append((float(state_s[0]), float(state_d[0])))

        return predicted


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