#!/usr/bin/env python3
"""
IMM (Interacting Multiple Model) Filter Node for Frenet-frame trajectory prediction.

Subscribes to /frenet_opp_state_vector  → (s, d, dt)
Publishes    to /imm_path               → nav_msgs/Path in "map" frame

Forward-propagates 10 timesteps × 0.25 s = 2.5 s ahead.

Coasting behaviour (silence measured from the last real observation, on the node clock):
  0 – COAST_START_S    : grace — the last published path stands.
  COAST_START_S – COAST_GIVEUP_S : coast — a timer runs predict-only IMM steps
                                   (covariance grows, no measurement) and republishes
                                   the look-ahead.
  ≥ COAST_GIVEUP_S     : give up — freeze the last predicted path and keep republishing
                                   it (refreshed stamp) so downstream consumers see a
                                   held path rather than a stale one.
The first real observation after any coast reinitialises the filters (forced by the
_coasting/_gave_up flags, and also caught by the existing dt > REINIT_THRESHOLD path).
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from racetrack_utilities.racetrack_utilities import RacetrackUtilities

from filterpy.kalman import KalmanFilter

from typing import Optional, List, Tuple


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
PREDICT_DT = 0.25          # seconds per forward-prop step (also the coast tick period)
N_STEPS    = 10            # number of steps to forward-propagate
REINIT_THRESHOLD = 2.0     # seconds; reinitialize KF if dt gap is too large

# Coast / hold thresholds (seconds of silence since the last real observation)
COAST_START_S  = 1.0   # begin coasting after this much silence
COAST_GIVEUP_S = 20.0                # stop advancing after this much silence, then hold


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
    f.R = np.eye(1) * 0.2
    f.Q = np.eye(3) * 0.2  # scaled for CV model
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
    f.R = np.eye(1) * 0.2
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
    f.P = np.eye(2) * 0.02
    f.R = np.eye(1) * 0.0025
    f.Q = np.eye(2) * 0.0025
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
    f.P = np.eye(2) * 0.02
    f.R = np.eye(1) * 0.0025
    f.Q = np.eye(2) * 0.0025
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
        s_mu0 = np.array([0.8, 0.2])
        s_M   = np.array([[0.99, 0.01],
                           [0.10, 0.90]])
        self.s_imm = IMMAxis(s_filters, s_mu0, s_M, axis='s')

        # build d-axis IMM
        d_filters = [
            make_d_cd_filter(dt0),   # model 0: constant d
            make_d_cv_filter(dt0),   # model 1: constant d-velocity
        ]
        d_mu0 = np.array([0.5, 0.5])
        d_M   = np.array([[0.95, 0.05],
                           [0.1, 0.9]])
        self.d_imm = IMMAxis(d_filters, d_mu0, d_M, axis='d')

        self._initialized = False
        self._last_s      = None
        self._last_d      = None

        self.last_ego_s   = None
        self.last_ego_d   = None

        # coast / hold state
        self._last_obs_time = None    # node-clock stamp of the last real observation
        self._coasting      = False   # True while running predict-only steps
        self._gave_up       = False   # True once past COAST_GIVEUP_S (holding last path)
        self._frozen_path   = None    # the Path held/republished after giving up
        self._last_path     = None    # most recently published prediction (freeze source)

        # ROS2 I/O
        self.sub = self.create_subscription(
            Float64MultiArray, '/frenet_opp_state_vector', self._cb, 10)
        self.pub = self.create_publisher(Path, '/imm_path', 10)

        self.cov_pub = self.create_publisher(Marker, '/imm_cov', 10)
        
        self.opp_pos_pub = self.create_publisher(Marker, '/starting_points', 10)
        
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)

        self.ego_ahead = False

        self.ego_ahead_pub = self.create_publisher(Bool, '/ego_ahead', 10)

        # Wall-clock timer that drives coasting during observation silence.
        self.coast_timer = self.create_timer(PREDICT_DT, self._coast_tick)
    
        self.get_logger().info('IMMFilterNode ready.')

    # ──────────────────────────────────────────────────────────────────────
    # Small state helpers
    # ──────────────────────────────────────────────────────────────────────
    def ego_odom_cb(self, msg : Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.last_ego_s, self.last_ego_d = self.racetrack_utilities.convert_to_frenet(x, y)

        if not self._last_s:
            self.ego_ahead = False
            return

        if self.last_ego_s >= self._last_s:
            self.ego_ahead = True
        else:
            self.ego_ahead = False


        self.ego_ahead_pub.publish(Bool(data=self.ego_ahead))
        
    def _reinit_from_obs(self, s, d):
        # type: (float, float) -> None
        """Cold-restart both IMM axes from a fresh (s, d) observation."""
        self.s_imm.reinitialize(np.array([s, 0.0, 0.0]))
        self.d_imm.reinitialize(np.array([d, 0.0]))
        self._last_s = s
        self._last_d = d


    def _clear_coast_state(self):
        # type: () -> None
        """Return to normal operation: drop any coast/hold bookkeeping."""
        self._coasting    = False
        self._gave_up     = False
        self._frozen_path = None

    def publish_sampled_points(self, s_mean, s_std, d_mean, d_std):
        s_values = np.random.normal(loc=s_mean, scale=s_std, size=10)
        d_values = np.random.normal(loc=d_mean, scale=min(1/2, d_std), size=10)
        d_values = np.clip(d_values, -0.8, 0.8)
        frenet_points = [(s_values[i], d_values[i]) for i in range(s_values.shape[0])]

        filtered_points_frenet = []
        for point in frenet_points:
            if point[0] >= self._last_s and point[0] <= (self.last_ego_s - 0.1*(self.last_ego_s - self._last_s)):
                filtered_points_frenet.append(point)
        
        cartesian_points = [tuple(self.racetrack_utilities.convert_to_cartesian(*frenet_point)) for frenet_point in filtered_points_frenet]

        ego_x, ego_y = self.racetrack_utilities.convert_to_cartesian(self._last_s, self._last_d)


        if len(cartesian_points) > 3:
            cartesian_points.sort(key=lambda point : (point[0] - ego_x)**2 + (point[1] - ego_y)**2)
            cartesian_points = cartesian_points[0:3]

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "opponent_state_estimate"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.points =  []
    
        for x, y in cartesian_points:
            new_point = Point()
            new_point.x = x
            new_point.y = y
            new_point.z = 0.0
            marker.points.append(new_point)

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        self.opp_pos_pub.publish(marker)
        
        

        

    def publish_covariance(self):
        # variance of s and variance of d, then randomly sample, since s and d are not  correlated
        cov_matrices_s = [filt.P for filt in self.s_imm.filters]
        cov_matrices_d = [filt.P for filt in self.d_imm.filters]
        states_s = self.s_imm.xs
        states_d = self.d_imm.xs

        # get belief distribution over the models
        mu_s = self.s_imm.mu
        mu_d = self.d_imm.mu

        avg_cov_mat_s = np.zeros_like(cov_matrices_s[0])
        avg_cov_mat_d = np.zeros_like(cov_matrices_d[0])

        s_mean = 0.0
        d_mean = 0.0

        # calculate average s / d covariance matrix (weighted based on mu vector) and then find standard deviation
        for i in range(len(cov_matrices_s)):
            avg_cov_mat_s += mu_s[i] * cov_matrices_s[i]
            avg_cov_mat_d += mu_d[i] * cov_matrices_d[i]
            s_mean += mu_s[i] * states_s[i][0]
            d_mean += mu_d[i] * states_d[i][0]

        # get 95% confidence interval bounds
        s_std = np.sqrt(avg_cov_mat_s[0, 0])
        d_std = np.sqrt(avg_cov_mat_d[0, 0])

        # empirical rule from high school stats lol
        limits = np.array([
            [s_mean - 2 * s_std, s_mean + 2 * s_std],
            [d_mean - 2 * d_std, d_mean + 2 * d_std]
        ])

        def clamp(s, d):
            s = s % self.racetrack_utilities.arclength
            d = np.minimum(np.maximum(d, -1), 1)
            return s, d

        

        boundary_points = [
            clamp(s_mean - 2 * s_std, d_mean - 2 * d_std),
            clamp(s_mean, d_mean - 2 * d_std),
            clamp(s_mean + 2 * s_std, d_mean - 2 * d_std),
            clamp(s_mean - 2 * s_std, d_mean),
            clamp(s_mean + 2 * s_std, d_mean),
            clamp(s_mean - 2 * s_std, d_mean + 2 * d_std),
            clamp(s_mean, d_mean + 2 * d_std),
            clamp(s_mean + 2 * s_std, d_mean + 2 * d_std)
        ]

        boundary_points_cartesian = [
            list(self.racetrack_utilities.convert_to_cartesian(*point)) for point in boundary_points 
        ]

        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "opponent_state_estimate"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.points =  []
        
        for x, y in boundary_points_cartesian:
            new_point = Point()
            new_point.x = x
            new_point.y = y
            new_point.z = 0.0
            marker.points.append(new_point)

        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        self.cov_pub.publish(marker)
        self.publish_sampled_points(s_mean, s_std, d_mean, d_std)

        return limits


    # ──────────────────────────────────────────────────────────────────────
    # Observation callback
    # ──────────────────────────────────────────────────────────────────────
    def _cb(self, msg):
        # type: (Float64MultiArray) -> None
        s, d, dt = msg.data[0], msg.data[1], msg.data[2]
        z_s = np.array([[s]])
        z_d = np.array([[d]])
        now = self.get_clock().now()

        # first observation: cold-start
        if not self._initialized:
            self._reinit_from_obs(s, d)
            self._initialized   = True
            self._last_obs_time = now
            self.get_logger().info('IMM initialised — s={:.3f}, d={:.3f}'.format(s, d))
            return

        # Reinitialise if the reported gap is large, OR if we have been coasting /
        # holding — in either case the live filter state is stale relative to this
        # fresh observation, so we discard it and cold-restart.
        if dt > REINIT_THRESHOLD or self._coasting or self._gave_up:
            if self._coasting or self._gave_up:
                reason = 'coast recovery'
            else:
                reason = 'large dt={:.2f}s'.format(dt)
            self.get_logger().warn('Reinitialising IMM filters ({}).'.format(reason))
            self._reinit_from_obs(s, d)
            self._clear_coast_state()
            self._last_obs_time = now
            return

        # normal update
        x_s, _ = self.s_imm.update(z_s, dt)
        x_d, _ = self.d_imm.update(z_d, dt)

        self._last_s        = x_s[0]
        self._last_d        = x_d[0]
        self._last_obs_time = now
        self._clear_coast_state()

        self._last_path = self._publish_prediction(x_s, x_d, verbose=True)
        self.publish_covariance()

    # ──────────────────────────────────────────────────────────────────────
    # Coast timer: runs every PREDICT_DT, active only during observation silence
    # ──────────────────────────────────────────────────────────────────────
    def _coast_tick(self):
        # type: () -> None
        if not self.ego_ahead: return
        if not self._initialized or self._last_obs_time is None:
            return

        now     = self.get_clock().now()
        silence = (now - self._last_obs_time).nanoseconds * 1e-9

        # Observations are still arriving (or within grace) — nothing to do; the
        # observation callback owns publishing in this regime.
        if silence < COAST_START_S:
            return

        # ── Coast: predict-only IMM step (no measurement). Covariance grows. ──
        if silence < COAST_GIVEUP_S:
            dt = PREDICT_DT if self._coasting else COAST_START_S
            self._coasting = True
            x_s, _ = self.s_imm.predict(dt)
            x_d, _ = self.d_imm.predict(dt)
            self._last_s = x_s[0]
            self._last_d = x_d[0]
            self._last_path = self._publish_prediction(x_s, x_d, verbose=False)
            self.publish_covariance()
            self.get_logger().info(
                'coasting (silence={:.2f}s) s={:.2f} d={:.2f}'.format(
                    silence, float(x_s[0]), float(x_d[0])),
                throttle_duration_sec=1.0)
            return

        # ── Give up: freeze the last predicted path and keep republishing it. ──
        if not self._gave_up:
            self._gave_up     = True
            #   self._frozen_path = self._last_path
            self._last_s = None
            self._last_d = None
            self._frozen_path = None
            self.get_logger().warn(
                'Coast bound reached (silence={:.2f}s) — holding last path.'.format(silence))

        if self._frozen_path is not None:
            # Refresh the stamp so the held path stays "live" for consumers.
            self._frozen_path.header.stamp = now.to_msg()
            self.pub.publish(self._frozen_path)

    # ──────────────────────────────────────────────────────────────────────
    # Prediction → Path publishing
    # ──────────────────────────────────────────────────────────────────────
    def _publish_prediction(self, x_s, x_d, verbose=True):
        # type: (np.ndarray, np.ndarray, bool) -> Path
        """Forward-propagate, build the Path, publish it, and return the message."""
        predicted = self._forward_propagate(x_s, x_d)

        if verbose:
            print('\n── IMM forward-propagated path ──')
            print('  {:>4}  {:>6}  {:>10}  {:>10}'.format('step', 't (s)', 's', 'd'))
            for k, (ps, pd) in enumerate(predicted):
                t = (k + 1) * PREDICT_DT
                print('  {:>4}  {:>6.2f}  {:>10.4f}  {:>10.4f}'.format(k + 1, t, ps, pd))

        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for ps, pd in predicted:
            x_cart, y_cart = self.racetrack_utilities.convert_to_cartesian(ps, pd)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x_cart
            pose.pose.position.y = y_cart
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.pub.publish(path_msg)
        return path_msg

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