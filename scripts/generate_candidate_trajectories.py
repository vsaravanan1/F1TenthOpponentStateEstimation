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
from scipy.interpolate import make_smoothing_spline
from racetrack_utilities.racetrack_utilities import RacetrackUtilities
from rrt_planner.rrt_planner import RRTStarPlanner


class OpponentIntentPredictor(Node):
    def __init__(self):
        super().__init__('opponent_intent_predictor')

        # ---- parameters -------------------------------------------------
        self.path_resolution = 40
        self.plan_period = 0.1            # timer rate; also the max publish rate
        self.marker_lifetime = 3.5 * self.plan_period   # expire fast, never linger
        self.max_start_age = 0.5          # drop starting points older than this [s]
        self.opponent_velocity = 2.5
        self.ego_velocity = 2.0
        self.d_goals = np.linspace(-0.8, 0.8, 5)
        self.d_clear = 0.5
        self.ego_d_candidates = np.linspace(-0.9, 0.9, 5)
        self.d_limit = 0.8
        self.smoothing_lam = 1      # make_smoothing_spline lam (t normalised to [0, 1])
        self.n_fit_points = 20       # RRT polyline is resampled to this many points (must be >= 5)
        self.max_s_clearance = 6.0

        self.rutil = RacetrackUtilities("/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv")
        self.planner = RRTStarPlanner(max_iter=500, d_limit=self.d_limit, d_clear=self.d_clear, time_budget=0.030)

        self.ego_state = (0.0, 0.0, 0.0)
        self.ego_data_received = False
        self.latest_points = None
        self.latest_stamp = None
        self.points_seq = 0
        self.planned_seq = 0

        # depth-1 queue: if we fall behind we skip to the newest message
        # instead of chewing through a backlog of stale ones.
        latest_qos = QoSProfile(depth=1,
                                history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        # Separate callback groups + MultiThreadedExecutor so that
        # planning never blocks odom / starting-point reception.
        io_group = MutuallyExclusiveCallbackGroup()
        plan_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Odometry, '/ego_racecar/odom',
                                 self.ego_odom_callback, latest_qos,
                                 callback_group=io_group)
        self.create_subscription(Marker, '/starting_points',
                                 self.starting_points_cb, latest_qos,
                                 callback_group=io_group)
        self.create_timer(self.plan_period, self.plan_timer_cb,
                          callback_group=plan_group)

        self.threat_splines_pubs = [
            self.create_publisher(MarkerArray, '/predicted_opponent_splines_0', 10),
            self.create_publisher(MarkerArray, '/predicted_opponent_splines_1', 10),
            self.create_publisher(MarkerArray, '/predicted_opponent_splines_2', 10),
        ]
        self.ego_lane_possibilities_pub = self.create_publisher(
            MarkerArray, '/ego_lane_possibilities', 10)

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def ego_odom_callback(self, msg):
        p = msg.pose.pose.position
        self.ego_state = (p.x, p.y, self.quat_to_yaw(msg.pose.pose.orientation))
        self.ego_data_received = True

    def starting_points_cb(self, msg: Marker):
        self.latest_points = [(p.x, p.y) for p in msg.points]
        self.latest_stamp = msg.header.stamp
        self.points_seq += 1

    # ------------------------------------------------------------------
    # Planning 
    # ------------------------------------------------------------------
    
    def plan_timer_cb(self):
        if not self.ego_data_received or self.latest_points is None:
            return
        if self.points_seq == self.planned_seq:
            return                        
        self.planned_seq = self.points_seq

        points = self.latest_points

        if len(points) == 0: return

        ex, ey, _ = self.ego_state       

        s_ego, d_ego = self.rutil.convert_to_frenet(ex, ey)
        starts = []
        for (x, y) in points[:3]:
            s_opp, d_opp = self.rutil.convert_to_frenet(x, y)
            starts.append((s_opp, d_opp))

        starts = np.asarray(starts, dtype=float)

        threat_paths, opponent_time_paths, max_ot_time = self.generate_opponent_splines_rrt(
            starts, s_ego, d_ego)
        ego_time_paths = self.generate_ego_lane_possibilities(s_ego, d_ego, max_ot_time)

        self.publish_threat_markers(threat_paths)

        best_ego_idx = self.find_best_defense_trajectory(opponent_time_paths, ego_time_paths)
        if best_ego_idx is not None:
            best = ego_time_paths[best_ego_idx]
            ego_cart = self.rutil.convert_to_cartesian_arr(best[:, 1], best[:, 2])
            self.publish_ego_lane_possibilities([ego_cart])


    def get_rrt_conditions(self, starts, s_ego, d_ego, goals):
        ego_pose_frenet = np.array([s_ego, d_ego])
        distances = ego_pose_frenet[0] - starts[:, 0]
        opp_ot_times = distances/(self.opponent_velocity - self.ego_velocity)
        s_clearances = np.minimum((self.ego_velocity) * opp_ot_times, self.max_s_clearance)
        goal_sets = []
        for s_clear in s_clearances:
            goal_sets.append(goals + np.array([s_clear + 1.0, 0.0]))
        
        return {
            "s_clearances": s_clearances,
            "goals": np.array(goal_sets),
            "opp_ot_times": opp_ot_times
        }

                
    def generate_opponent_splines_rrt(self, starts, s_ego, d_ego):
        self.planner.set_obstacles([(s_ego, d_ego)])
        rrt_goals = np.column_stack((np.full(len(self.d_goals), s_ego), self.d_goals))

        threat_paths = []
        opponent_time_paths = []

        rrt_conditions_dict = self.get_rrt_conditions(starts, s_ego, d_ego, rrt_goals)
        s_clearances = rrt_conditions_dict["s_clearances"]
        goals = rrt_conditions_dict["goals"]
        max_ot_time = max(rrt_conditions_dict["opp_ot_times"])

        for i, (s_opp, d_opp) in enumerate(starts):
            point_paths = []
            point_time_paths = []

            # opponent already ahead of the goal -> nothing meaningful to plan
            if s_opp < s_ego - 0.5:
                courses = self.planner.plan((s_opp, d_opp), goals[i], s_clearances[i], step=0.6)
                for course in courses:
                    if course is None:
                        continue
                    time_path = self.path_to_time_trajectory(course, self.opponent_velocity)
                    if time_path is None:
                        continue
                    cart = self.rutil.convert_to_cartesian_arr(time_path[:, 1], time_path[:, 2])
                    point_paths.append(cart.tolist())
                    point_time_paths.append(time_path)

            threat_paths.append(point_paths)
            opponent_time_paths.append(point_time_paths)

        return threat_paths, opponent_time_paths, max_ot_time

    def generate_ego_lane_possibilities(self, s_ego, d_ego, max_ot_time):
        """Constant-speed ego paths with a smooth lateral shift over the 2nd half."""
        horizon = min(max_ot_time * self.ego_velocity, self.max_s_clearance)
        T = horizon / self.ego_velocity
        t = np.linspace(0.0, T, self.path_resolution)
        u = np.clip((t / T - 0.5) / 0.5, 0.0, 1.0)
        blend = u * u * (3.0 - 2.0 * u)          # smoothstep
        s = s_ego + self.ego_velocity * t

        return [np.column_stack((t, s, d_ego + (d_traj - d_ego) * blend))
                for d_traj in self.ego_d_candidates]


    def path_to_time_trajectory(self, P, velocity):
        """RRT waypoints (N,2) -> (path_resolution, 3) array of [t, s, d] at
        constant speed, using smoothing splines to smooth the RRT path."""
        P = np.asarray(P, dtype=float)
        seg = np.hypot(*np.diff(P, axis=0).T)
        P = P[np.concatenate(([True], seg > 1e-6))]
        if len(P) < 2:
            return None

        u = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))))
        u_fit = np.linspace(0.0, u[-1], max(5, self.n_fit_points))
        s_fit = np.interp(u_fit, u, P[:, 0])
        d_fit = np.interp(u_fit, u, P[:, 1])

        t_param = np.linspace(0.0, 1.0, len(u_fit))
        spline_s = make_smoothing_spline(t_param, s_fit, lam=self.smoothing_lam)
        spline_d = make_smoothing_spline(t_param, d_fit, lam=self.smoothing_lam)

        uu = np.linspace(0.0, 1.0, 100)
        Q = np.column_stack((spline_s(uu), spline_d(uu)))
        Q[:, 1] = np.clip(Q[:, 1], -self.d_limit, self.d_limit)

        arc = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))))
        if arc[-1] < 1e-6:
            return None
        t = arc / velocity
        ts = np.linspace(0.0, t[-1], self.path_resolution)
        return np.column_stack((ts, np.interp(ts, t, Q[:, 0]), np.interp(ts, t, Q[:, 1])))

    # ------------------------------------------------------------------
    # Collision / selection
    # ------------------------------------------------------------------
    def find_best_defense_trajectory(self, opponent_paths, ego_paths):
        """Ego trajectory with the smallest average TTC over all colliding
        (ego, opponent) pairs across all starting points."""
        if len(opponent_paths) == 0 or len(ego_paths) == 0:
            return None

        average_ttc = []
        for ego_path in ego_paths:
            collision_times = []
            for starting_point_paths in opponent_paths:
                for opponent_path in starting_point_paths:
                    ttc = self.find_time_to_collision(opponent_path, ego_path)
                    if ttc is not None:
                        collision_times.append(ttc)
            average_ttc.append(np.mean(collision_times) if collision_times else np.inf)

        if all(np.isinf(v) for v in average_ttc):
            return None
        return int(np.argmin(average_ttc))

    def find_time_to_collision(self, opponent_path, ego_path, n_samples=150):
        if len(opponent_path) == 0 or len(ego_path) == 0:
            return None

        ot, et = opponent_path[:, 0], ego_path[:, 0]
        start_time = max(ot[0], et[0])
        end_time = min(ot[-1], et[-1])
        if start_time > end_time:
            return None

        times = np.linspace(start_time, end_time, n_samples)
        ds = np.interp(times, ot, opponent_path[:, 1]) - np.interp(times, et, ego_path[:, 1])
        dd = np.interp(times, ot, opponent_path[:, 2]) - np.interp(times, et, ego_path[:, 2])

        hits = np.nonzero((np.abs(ds) < 0.3) & (np.abs(dd) < 0.25))[0]
        if len(hits) == 0:
            return None
        return float(times[hits[0]])

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _line_marker(self, ns, idx, rgb, path, stamp):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = idx
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 0.8
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        for pt in path:
            p = Point()
            p.x, p.y, p.z = float(pt[0]), float(pt[1]), 0.0
            marker.points.append(p)
        return marker

    def publish_threat_markers(self, threat_paths):
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        stamp = self.get_clock().now().to_msg()
        for point_idx, point_paths in enumerate(threat_paths[:3]):
            arr = MarkerArray()
            for idx, path in enumerate(point_paths):
                arr.markers.append(self._line_marker(
                    f"opponent_threats_{point_idx}", idx, colors[point_idx], path, stamp))
            self.threat_splines_pubs[point_idx].publish(arr)

    def publish_ego_lane_possibilities(self, ego_paths):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for idx, path in enumerate(ego_paths):
            arr.markers.append(self._line_marker(
                "ego_lane_possibilities", idx, (1.0, 1.0, 0.0), path, stamp))
        self.ego_lane_possibilities_pub.publish(arr)

    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = OpponentIntentPredictor()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()