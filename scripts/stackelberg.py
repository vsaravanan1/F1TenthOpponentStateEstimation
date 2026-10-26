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
from scipy.interpolate import make_interp_spline, splprep, splev, make_splprep
from scipy.optimize import bisect, fsolve
from typing import Optional


# marker array with diverse trajectories contains 40 points for each traj
# each path is an n by 2 array in cartesian coordinate
def time_to_collision(opponent_path : np.ndarray, ego_path : np.ndarray, opponent_velocity : float, ego_velocity : float) -> Optional[float]:
    u = np.linspace(0, 1, 1000)
    opp_tck, _ = splprep([opponent_path[:,0], opponent_path[:,1]], s=0.2, u=u)
    ego_tck, _ = splprep([ego_path[:, 0], ego_path[:, 1]], s=0.2, u=u)

    opp_coordinates = splev(u, opp_tck, der=0)
    ego_coordinates = splev(u, ego_tck, der=0)
    
    opp_diffs = np.diff(opp_coordinates, axis=0)
    ego_diffs = np.diff(ego_coordinates, axis=0)

    opp_s = np.insert(np.cumsum(np.linalg.norm(opp_diffs, axis=1)), 0.0, 0.0)
    ego_s = np.insert(np.cumsum(np.linalg.norm(ego_diffs, axis=1)), 0.0, 0.0)
    
    opp_times = opp_s / opponent_velocity
    ego_times = ego_s / ego_velocity

    horizon = max(ego_times[-1], opp_times[-1])

    time_opp_tck, _ = splprep([opp_coordinates[:,0], opp_coordinates[:,1]], u=opp_times)
    time_ego_tck, _ = splprep([ego_coordinates[:,0], ego_coordinates[:,1]],u=ego_times)

    THRESHOLD = 0.2

    sample_times = np.linspace(0, horizon, 1000)
    opp_time_coordinates = splev(sample_times, time_opp_tck, der=0)
    ego_time_coordinates = splev(sample_times, time_ego_tck, der=0)

    ego_opp_dists = np.linalg.norm(opp_time_coordinates - ego_time_coordinates, axis=1)
    intersect_mask = ego_opp_dists < THRESHOLD
    if np.any(intersect_mask):
        intersect_idx = np.where(intersect_mask)[0]
        return sample_times[intersect_idx]
    else:
        return None


def t_safe(opponent_path : np.ndarray, ego_path : np.ndarray, opponent_velocity : float) -> float:
    t_delay = 0.15

    mu = 0.4 # arbitrary mu value thats purposefully very low to make t_safe real safe
    g = 9.81
    v_o = opponent_velocity
    k_o = 

    # (ego_width + opp_width)/2 + d_buf
    # ego width == opp width == 0.2794
    # d_buf == 0.1 m so we just round to 0.4
    # Subject to change
    W = 0.4 

    if (mu * g)**2 - (v_o**2 * k_o)**2 < 0:
        return np.inf

    a_lon = np.sqrt((mu * g)**2 - (v_o**2 * k_o)**2)
    a_lat = (mu * g) - (v_o**2 * np.abs(k_o))

    t_safe = min(v_o/a_lon, np.sqrt((2*W) / a_lat))
    return t_delay + t_safe


def raceline_prog_opp(opponent_path : np.ndarray, opponent_velocity : float) -> float:
    t_horiz = 2 # look ahead 2 seconds -- arbitrary val
    s_start = # opponent path curr position
    s_end = # opponent path after

    s_fix_start = s_start # same starting position
    s_fix_end = s_fix_start + (opponent_velocity * t_horiz) # all velocity contributed to s (no lateral movement change)
    
    r = (s_end - s_start) / (s_fix_start - s_fix_end)

    return r    


def raceline_prog_ego(ego_path : np.ndarray, ego_velocity : float) -> float:
    t_horiz = 2 # look ahead 2 seconds -- arbitrary val
    s_start = # opponent path curr position
    s_end = # opponent path after

    s_fix_start = s_start # same starting position
    s_fix_end = s_fix_start + (ego_velocity * t_horiz) # all velocity contributed to s (no lateral movement change)
    
    r = (s_end - s_start) / (s_fix_start - s_fix_end)

    return r


def traj_hit_wall_opponent(opponent_path : np.ndarray) -> bool:
    
    pass
    
    
def traj_hit_wall_ego(ego : np.ndarray) -> bool:
    
    pass


def rho(opponent_path : np.ndarray, ego_path : np.ndarray, opponent_velocity : float, ego_velocity : float) -> float:
    t_0 = 1
    t = time_to_collision(opponent_path, ego_path, opponent_velocity, ego_velocity) / t_safe(opponent_path, ego_path, opponent_velocity)

    return min(1, (t_0/t)) # assuming p (exp) is equal to 1. Subject to change


def pi(opponent_path : np.ndarray, ego_path : np.ndarray, opponent_velocity : float, ego_velocity : float) -> float:
    t_0 = 1
    t = time_to_collision(opponent_path, ego_path, opponent_velocity, ego_velocity) / t_safe(opponent_path, ego_path, opponent_velocity)

    return max(0, np.log(t_0/t))


def pick_trajectory(t = time_to_collision(opponent_path, ego_path, opponent_velocity, ego_velocity) / t_safe(opponent_path, ego_path, opponent_velocity)):
    
    # ego cost function weights
    w_block =
    w_reck =

    # opponent cost function weights
    w_yield = 
    w_crash = 
