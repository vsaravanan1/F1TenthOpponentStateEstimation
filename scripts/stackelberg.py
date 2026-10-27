#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from scipy.interpolate import make_smoothing_spline
from racetrack_utilities.racetrack_utilities import RacetrackUtilities
from rrt_planner.rrt_planner import RRTStarPlanner
from scipy.interpolate import make_interp_spline, splprep, splev
from scipy.optimize import bisect, fsolve
from typing import Optional


# f1tenth_gym vehicle params ('length', 'width'); same car for ego and opponent
CAR_LENGTH = 0.58
CAR_WIDTH = 0.31

COLLISION_BUF = 0.05 # grows each footprint, so cars closer than 0.1 m (d_buf) count as colliding. Subject to change
TTC_DT = 0.01 # [s] time step of the collision search

M = 1000.0 # payoff of a trajectory that hits the wall
TAU_MIN = 1e-3 # caps pi at ln(1/TAU_MIN) ~ 6.9, so no pair scores below the -M wall floor


# marker array with diverse trajectories contains 40 points for each traj
# each path is an n by 2 array in cartesian coordinate
# (n by 3 [t, s, d] frenet paths are accepted too, see path_to_xy)
def path_to_xy(path : np.ndarray, rutil : RacetrackUtilities) -> np.ndarray:
    # (N, 2) cartesian [x, y] passes through; (N, 3) [t, s, d] is mapped to cartesian
    path = np.asarray(path, dtype=float)
    if path.shape[1] == 3:
        return rutil.convert_to_cartesian_arr(path[:, 1], path[:, 2])
    return path[:, :2]


def path_times(xy : np.ndarray, velocity : float) -> np.ndarray:
    # time at each point when the path is driven at constant velocity
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))) / velocity


def path_heading(xy : np.ndarray) -> np.ndarray:
    dxy = np.gradient(xy, axis=0)
    return np.arctan2(dxy[:, 1], dxy[:, 0])


def path_t_s(path : np.ndarray, velocity : float, rutil : RacetrackUtilities):
    """Time and along-track position at each point of path, driving its cartesian
    geometry at constant velocity (same assumption as time_to_collision). s is
    metres along the centerline, unwrapped across the finish line."""
    xy = path_to_xy(path, rutil)
    lap = rutil.arclength

    s = rutil.convert_to_frenet_arr(xy)[:, 0]
    s = s - lap * np.concatenate(([0.0], np.cumsum(np.round(np.diff(s) / lap)))) # unwrap across the finish line (np.unwrap's period= needs numpy >= 1.21)

    return path_times(xy, velocity), s


def time_to_collision(opponent_path : np.ndarray, ego_path : np.ndarray, opponent_velocity : float, ego_velocity : float, rutil : RacetrackUtilities) -> float:
    """CostFunctions Eq. 2: first time t in [0, H] at which the two cars collide,
    each driving its path at constant velocity, or np.inf if they never do. H is
    the shorter path's duration -- past it, that car's position is unknown.

    "Collide" means the two CAR_LENGTH x CAR_WIDTH footprints (grown by
    COLLISION_BUF, each at its own heading) overlap. A single centre-distance
    epsilon can't do this: side-by-side cars touch at 0.31 m between centres,
    nose-to-tail ones at 0.58 m.
    """
    opp_xy = path_to_xy(opponent_path, rutil)
    ego_xy = path_to_xy(ego_path, rutil)
    opp_t = path_times(opp_xy, opponent_velocity)
    ego_t = path_times(ego_xy, ego_velocity)

    sample_times = np.arange(0.0, min(opp_t[-1], ego_t[-1]), TTC_DT)

    dx = np.interp(sample_times, opp_t, opp_xy[:, 0]) - np.interp(sample_times, ego_t, ego_xy[:, 0])
    dy = np.interp(sample_times, opp_t, opp_xy[:, 1]) - np.interp(sample_times, ego_t, ego_xy[:, 1])
    ego_h = np.interp(sample_times, ego_t, np.unwrap(path_heading(ego_xy)))
    phi = np.interp(sample_times, opp_t, np.unwrap(path_heading(opp_xy))) - ego_h # opponent heading relative to ego

    # opponent centre in the ego's frame
    lon = dx * np.cos(ego_h) + dy * np.sin(ego_h)
    lat = -dx * np.sin(ego_h) + dy * np.cos(ego_h)

    # separating axis test: the footprints overlap iff their projections overlap on all four rectangle axes
    half_l = CAR_LENGTH / 2 + COLLISION_BUF
    half_w = CAR_WIDTH / 2 + COLLISION_BUF
    abs_cos, abs_sin = np.abs(np.cos(phi)), np.abs(np.sin(phi))
    reach_l = half_l + half_l * abs_cos + half_w * abs_sin # combined half-extent along either car's length axis
    reach_w = half_w + half_l * abs_sin + half_w * abs_cos # combined half-extent along either car's width axis
    overlap = ((np.abs(lon) <= reach_l) & (np.abs(lat) <= reach_w) &
               (np.abs(lon * np.cos(phi) + lat * np.sin(phi)) <= reach_l) &
               (np.abs(-lon * np.sin(phi) + lat * np.cos(phi)) <= reach_w))

    hits = np.flatnonzero(overlap)
    return float(sample_times[hits[0]]) if len(hits) else np.inf


def t_safe(opponent_path : np.ndarray, opponent_velocity : float, t_c : float, rutil : RacetrackUtilities) -> float:
    t_delay = 0.15

    mu = 1.0489 # f1tenth_gym mu -- the sim's actual friction, not a deliberately low one
    g = 9.81
    v_o = opponent_velocity

    xy = path_to_xy(opponent_path, rutil)
    t = path_times(xy, opponent_velocity)

    x, y = xy[:, 0], xy[:, 1]

    dx,  dy  = np.gradient(x, t),  np.gradient(y, t)
    ddx, ddy = np.gradient(dx, t), np.gradient(dy, t)
    denom = (dx**2 + dy**2) ** 1.5
    kappa = np.where(denom > 1e-9, (dx * ddy - dy * ddx) / denom, 0.0)
    k_o = np.interp(t_c, t, kappa) # curvature at P, where the opponent is at t_c

    # (ego_width + opp_width)/2 + d_buf
    # ego width == opp width == CAR_WIDTH (0.31)
    # d_buf == 0.1 m
    # Subject to change
    W = 0.41

    if v_o**2 * abs(k_o) >= mu * g:
        return np.inf

    a_lon = np.sqrt((mu * g)**2 - (v_o**2 * k_o)**2)
    a_lat = (mu * g) - (v_o**2 * np.abs(k_o))

    t_safe = min(v_o/a_lon, np.sqrt((2*W) / a_lat))
    return t_delay + t_safe


def normalized_t(opponent_path : np.ndarray, ego_path : np.ndarray, opponent_velocity : float, ego_velocity : float, rutil : RacetrackUtilities) -> float:
    tc = time_to_collision(opponent_path, ego_path, opponent_velocity, ego_velocity, rutil)

    if np.isinf(tc):
        return np.inf

    ts = t_safe(opponent_path, opponent_velocity, tc, rutil)

    if np.isinf(ts):
        return 0.0

    return tc / ts


def raceline_prog_opp(opponent_path : np.ndarray, opponent_velocity : float, rutil : RacetrackUtilities) -> float:
    t_horiz = 2 # look ahead 2 seconds -- arbitrary val
    t, s = path_t_s(opponent_path, opponent_velocity, rutil)
    t_horiz = min(t_horiz, t[-1]) # a path shorter than the look-ahead is scored over its own length
    s_start = s[0] # opponent path curr position
    s_end = np.interp(t_horiz, t, s) # opponent path after t_horiz

    s_fix_start = s_start # same starting position
    s_fix_end = s_fix_start + (opponent_velocity * t_horiz) # all velocity contributed to s (no lateral movement change)

    r = (s_end - s_start) / (s_fix_end - s_fix_start)

    return r


def raceline_prog_ego(ego_path : np.ndarray, ego_velocity : float, rutil : RacetrackUtilities) -> float:
    t_horiz = 2 # look ahead 2 seconds -- arbitrary val
    t, s = path_t_s(ego_path, ego_velocity, rutil)
    t_horiz = min(t_horiz, t[-1]) # a path shorter than the look-ahead is scored over its own length
    s_start = s[0] # ego path curr position
    s_end = np.interp(t_horiz, t, s) # ego path after t_horiz

    s_fix_start = s_start # same starting position
    s_fix_end = s_fix_start + (ego_velocity * t_horiz) # all velocity contributed to s (no lateral movement change)

    r = (s_end - s_start) / (s_fix_end - s_fix_start)

    return r


def traj_hit_wall(path : np.ndarray, rutil : RacetrackUtilities, margin : float = 0.0) -> bool:
    """True if the car's footprint leaves the track at any pose along path.

    path is (N, 2) cartesian [x, y] or (N, 3) [t, s, d].

    Each pose's footprint is a CAR_LENGTH x CAR_WIDTH rectangle, grown by margin
    on every side, centred on the pose and aligned with the path heading. The
    pose hits the wall if any corner or long-side midpoint falls outside
    -w_right(s) < d < w_left(s). The midpoints catch the inner wall of a tight
    corner, which can touch the side of the car between the corners.
    """
    xy = path_to_xy(path, rutil)

    heading = path_heading(xy)
    cos_h, sin_h = np.cos(heading)[:, None], np.sin(heading)[:, None]

    half_l = CAR_LENGTH / 2 + margin
    half_w = CAR_WIDTH / 2 + margin
    # left side front -> rear, then right side rear -> front
    body_x = np.array([half_l, 0.0, -half_l, -half_l, 0.0, half_l])
    body_y = np.array([half_w, half_w, half_w, -half_w, -half_w, -half_w])

    # (N, 6, 2) footprint points in the map frame
    footprint = xy[:, None, :] + np.stack((cos_h * body_x - sin_h * body_y,
                                           sin_h * body_x + cos_h * body_y), axis=-1)

    # nearest-centerline projection; safe on Spielberg since no two track sections come within 4 m
    sd = rutil.convert_to_frenet_arr(footprint.reshape(-1, 2))
    left_w, right_w = splev(sd[:, 0] / rutil.arclength, rutil.racetrack_widths_spline, der=0)

    d = sd[:, 1]
    return bool(np.any((d >= left_w) | (d <= -right_w)))


def traj_hit_wall_opponent(opponent_path : np.ndarray, rutil : RacetrackUtilities) -> bool:
    # no margin: padding the opponent would gate lines it can really drive to -M,
    # and the ego would then never defend against them
    return traj_hit_wall(opponent_path, rutil, margin=0.0)


def traj_hit_wall_ego(ego_path : np.ndarray, rutil : RacetrackUtilities) -> bool:
    # margin covers controller tracking error. Subject to change
    return traj_hit_wall(ego_path, rutil, margin=0.05)


def rho(tau : float) -> float:
    t_0 = 1
    t = max(tau, TAU_MIN)

    return min(1, (t_0/t)) # assuming p (exp) is equal to 1. Subject to change


def pi(tau : float) -> float:
    t_0 = 1
    if np.isinf(tau): # no interaction
        return 0.0
    t = max(tau, TAU_MIN)

    return max(0, np.log(t_0/t))


def pick_trajectory(ego_paths, opponent_paths, ego_velocity : float, opponent_velocity : float, rutil : RacetrackUtilities) -> int:
    """Stackelberg pick (CostFunctions Listing 1): the ego leads, the opponent
    best-responds to each ego trajectory, and the ego takes the trajectory that
    scores highest against that response. Returns an index into ego_paths.
    Needs at least one opponent trajectory."""

    # ego cost function weights
    w_block = 0.05 # a full block (rho = 1) is worth 5% progress; candidates' R differ by ~0.1-3%. Subject to change
    w_reck = 1.0 # pi(0.95) ~ 0.05, so a block at tau = 0.95 already costs what it's worth. Subject to change

    # opponent cost function weights
    w_yield = 0.05 # Subject to change
    w_crash = 1.0 # Subject to change

    # wall hits and progress depend on one trajectory only, so compute them once each
    ego_wall = [traj_hit_wall_ego(p, rutil) for p in ego_paths]
    opp_wall = [traj_hit_wall_opponent(p, rutil) for p in opponent_paths]
    ego_prog = [raceline_prog_ego(p, ego_velocity, rutil) for p in ego_paths]
    opp_prog = [raceline_prog_opp(p, opponent_velocity, rutil) for p in opponent_paths]

    n, m = len(ego_paths), len(opponent_paths)
    A = np.empty((n, m))
    B = np.empty((n, m))
    for i in range(n):
        for j in range(m):
            tau = normalized_t(opponent_paths[j], ego_paths[i], opponent_velocity, ego_velocity, rutil)
            A[i, j] = -M if ego_wall[i] else ego_prog[i] + w_block * rho(tau) - w_reck * pi(tau)
            B[i, j] = -M if opp_wall[j] else opp_prog[j] - w_yield * rho(tau) - w_crash * pi(tau)

    j_star = np.argmax(B, axis=1) # opponent's best response to each ego trajectory
    return int(np.argmax(A[np.arange(n), j_star]))
