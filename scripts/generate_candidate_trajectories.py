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
from stackelberg import pick_trajectory

# Number of sampled opponent starting points we accept per planning cycle.
NUM_START_POINTS = 5


class OpponentIntentPredictor(Node):
    def __init__(self):
        super().__init__('opponent_intent_predictor')

        # ---- parameters -------------------------------------------------
        self.path_resolution = 40
        self.plan_period = 0.1            # RRT* planning rate (gated on new starting points)
        self.marker_lifetime = 3.5 * self.plan_period   # expire fast, never linger
        self.max_start_age = 0.5          # drop starting points older than this [s]
        self.opponent_velocity = 2.5
        self.ego_velocity = 1.5
        self.d_goals = np.linspace(-0.8, 0.8, 5)
        self.d_clear = 0.5
        # +-0.65 still leaves < a car width (0.31 m) to the wall, so the outer lanes fully block,
        # and < 1% of them put the car's footprint into the wall; from +-0.75 up, short lanes mostly do
        self.ego_d_candidates = np.linspace(-0.65, 0.65, 5)
        self.d_limit = 0.8
        self.smoothing_lam = 0.5      # make_smoothing_spline lam (t normalised to [0, 1])
        self.n_fit_points = 40       # RRT polyline is resampled to this many points (must be >= 5)
        self.max_s_clearance = 5.0

        self.rutil = RacetrackUtilities("/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv")
        self.planner = RRTStarPlanner(max_iter=500, d_limit=self.d_limit, d_clear=self.d_clear,
                                      time_budget=0.2, step_target_hops=12, step_min=0.5, step_max=3.0)

        # EMA of the measured wall-time of a whole planning cycle [s], from reading
        # the poses to publishing. Both cars are projected forward by it so the
        # cycle plans (and publishes) for where they will be when it finishes,
        # not where they were when it started. Seeded with the RRT* worst case:
        # time_budget applies per plan() call, i.e. per starting point.
        self.plan_duration_est = self.planner.time_budget * NUM_START_POINTS

        self.ego_state = (0.0, 0.0, 0.0)
        self.ego_speed = 0.0              # measured, from odom
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

        # Two independent callback groups under a MultiThreadedExecutor so the
        # slow RRT* planning cycle never blocks odom / starting-point reception:
        #   io_group   -- odom + starting points (must stay fresh)
        #   plan_group -- the slow RRT* planning cycle (does all publishing)
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

        # One threat-spline publisher per sampled starting point.
        self.threat_splines_pubs = [
            self.create_publisher(MarkerArray, f'/predicted_opponent_splines_{i}', 10)
            for i in range(NUM_START_POINTS)
        ]
        # Chosen (Stackelberg pick) defensive spline only.
        self.ego_best_defensive_spline_pub = self.create_publisher(
            MarkerArray, '/ego_best_defensive_spline', 10)
        # Every candidate defensive spline (best highlighted green).
        self.ego_candidate_splines_pub = self.create_publisher(
            MarkerArray, '/ego_candidate_splines', 10)
        # Number of opponent overtaking splines generated this cycle.
        self.num_overtaking_splines_pub = self.create_publisher(
            Int32, '/num_overtaking_splines', 10)

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def ego_odom_callback(self, msg):
        p = msg.pose.pose.position
        self.ego_state = (p.x, p.y, self.quat_to_yaw(msg.pose.pose.orientation))
        self.ego_speed = msg.twist.twist.linear.x
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

        cycle_t0 = time.perf_counter()
        ex, ey, _ = self.ego_state
        ego_speed = self.ego_speed

        s_ego, d_ego = self.rutil.convert_to_frenet(ex, ey)

        # Project both cars forward (constant velocity, along s) by the expected
        # cycle duration: the ego lanes, the RRT* obstacle and goals, and the
        # opponent starts all describe the moment this cycle publishes. Without
        # it everything is published one cycle late and the car is already past
        # the start of its own lanes.
        lead = self.plan_duration_est
        s_ego += ego_speed * lead
        s_ego_goal = s_ego

        starts = []
        for (x, y) in points[:NUM_START_POINTS]:
            s_opp, d_opp = self.rutil.convert_to_frenet(x, y)
            starts.append((s_opp + self.opponent_velocity * lead, d_opp))

        starts = np.asarray(starts, dtype=float)

        threat_paths, opponent_time_paths, max_ot_time = self.generate_opponent_splines_rrt(
            starts, s_ego, d_ego, s_ego_goal)

        ego_time_paths = self.generate_ego_lane_possibilities(s_ego, d_ego, max_ot_time)

        num_overtaking_splines = self.publish_threat_markers(threat_paths)

        # Every overtaking path from every starting point is one opponent option.
        opponent_paths = [path for point_paths in opponent_time_paths for path in point_paths]
        best_ego_idx = None
        if opponent_paths and ego_time_paths:
            best_ego_idx = pick_trajectory(ego_time_paths, opponent_paths,
                                           self.ego_velocity, self.opponent_velocity, self.rutil)

        # Feed the measured cycle time into next cycle's projection (EMA).
        cycle_time = time.perf_counter() - cycle_t0
        self.plan_duration_est += 0.3 * (cycle_time - self.plan_duration_est)
        self.get_logger().info(
            f"planning cycle {1e3 * cycle_time:.0f} ms, projected ego {ego_speed * lead:.2f} m ahead "
            f"({ego_speed:.2f} m/s x {1e3 * lead:.0f} ms)", throttle_duration_sec=1.0)

        # Publish the defensive splines ONLY when overtaking splines were
        # produced this cycle (same trigger as the threat markers) and a
        # defensive lane was actually selected.
        if num_overtaking_splines > 0 and best_ego_idx is not None:
            ego_carts = [self.rutil.convert_to_cartesian_arr(p[:, 1], p[:, 2])
                         for p in ego_time_paths]
            # All candidates -> /ego_candidate_splines (best highlighted green).
            self.publish_ego_candidate_splines(ego_carts, best_ego_idx)
            # Best defence only -> /ego_best_defensive_spline.
            self.publish_ego_best_defensive_spline([ego_carts[best_ego_idx]])


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


    def generate_opponent_splines_rrt(self, starts, s_ego, d_ego, s_ego_goal):
        # The obstacle sits at s_ego and the goals are built around s_ego_goal
        # (plan_timer_cb passes the same projected position for both).
        self.planner.set_obstacles([(s_ego, d_ego)])
        rrt_goals = np.column_stack((np.full(len(self.d_goals), s_ego_goal), self.d_goals))

        threat_paths = []
        opponent_time_paths = []

        # Catch-up dynamics (opp_ot_times / s_clearances) use the gap between
        # the (projected) ego and opponent starts.
        rrt_conditions_dict = self.get_rrt_conditions(starts, s_ego, d_ego, rrt_goals)
        s_clearances = rrt_conditions_dict["s_clearances"]
        goals = rrt_conditions_dict["goals"]
        max_ot_time = max(rrt_conditions_dict["opp_ot_times"])

        for i, (s_opp, d_opp) in enumerate(starts):
            point_paths = []
            point_time_paths = []

            # opponent already ahead of the goal -> nothing meaningful to plan
            if s_opp < s_ego - 0.5:
                # One dedicated RRT* tree per goal: every reachable goal in this
                # start's goal set gets its own collision-free trajectory if one
                # exists. step omitted -> the planner picks a dynamic step per
                # goal, larger for goals that are farther away (see
                # RRTStarPlanner.plan / _dynamic_step).
                courses = self.planner.plan((s_opp, d_opp), goals[i], s_clearances[i])
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

    def _ego_lane_path(self, s_ego, d_ego, d_traj, max_ot_time):
        """One constant-speed ego lane as a (path_resolution, 3) [t, s, d] array,
        starting at (s_ego, d_ego) and shifting toward d_traj.

        The lateral blend is concave up (u**2) and begins at t=0 with no flat
        hold, so the divert is carried in the reference from the very start of
        the path (the downstream controller sees the offset immediately) rather
        than the previous ease-in/ease-out smoothstep that held d_ego for the
        first 30% of the horizon. Caller guarantees max_ot_time > 0."""
        horizon = min(max_ot_time * self.ego_velocity, self.max_s_clearance)
        T = horizon / self.ego_velocity
        t = np.linspace(0.0, T, self.path_resolution)
        u = t / T                                # no dead time: shift begins at t=0
        blend = u * u                            # concave-up lateral profile
        s = s_ego + self.ego_velocity * t
        return np.column_stack((t, s, d_ego + (d_traj - d_ego) * blend))

    def generate_ego_lane_possibilities(self, s_ego, d_ego, max_ot_time):
        """Constant-speed ego lanes fanning out to each candidate d, plus (at
        index 0) one that holds d_ego and only moves forward in s -- the
        non-deviating option pick_trajectory weighs defending against. Returns
        [] for a degenerate (non-positive) horizon so selection/publishing
        cleanly produce 'no defence'."""
        if max_ot_time <= 1e-3:
            return []
        return [self._ego_lane_path(s_ego, d_ego, d_traj, max_ot_time)
                for d_traj in [d_ego, *self.ego_d_candidates]]


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
        """Publish the opponent overtaking splines and their count. Returns the
        total number of overtaking splines generated this cycle."""
        # One distinct colour per starting point (NUM_START_POINTS total).
        colors = [
            (1.0, 0.0, 0.0),   # red
            (0.0, 1.0, 0.0),   # green
            (0.0, 0.0, 1.0),   # blue
            (1.0, 0.0, 1.0),   # magenta
            (0.0, 1.0, 1.0),   # cyan
        ]
        stamp = self.get_clock().now().to_msg()
        num_overtaking_splines = 0
        for point_idx, point_paths in enumerate(threat_paths[:NUM_START_POINTS]):
            arr = MarkerArray()
            for idx, path in enumerate(point_paths):
                arr.markers.append(self._line_marker(
                    f"opponent_threats_{point_idx}", idx, colors[point_idx], path, stamp))
            self.threat_splines_pubs[point_idx].publish(arr)
            num_overtaking_splines += len(point_paths)

        # Publish the total count of overtaking splines generated this cycle.
        self.num_overtaking_splines_pub.publish(Int32(data=num_overtaking_splines))
        return num_overtaking_splines

    def publish_ego_best_defensive_spline(self, ego_paths):
        """Publish the chosen (Stackelberg pick) defensive spline on
        /ego_best_defensive_spline."""
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for idx, path in enumerate(ego_paths):
            arr.markers.append(self._line_marker(
                "ego_best_defensive_spline", idx, (1.0, 1.0, 0.0), path, stamp))
        self.ego_best_defensive_spline_pub.publish(arr)

    def publish_ego_candidate_splines(self, ego_paths, best_idx=None):
        """Publish every candidate defensive spline on /ego_candidate_splines.
        The best defence (Stackelberg pick) is drawn in green, the rest yellow."""
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for idx, path in enumerate(ego_paths):
            rgb = (0.0, 1.0, 0.0) if idx == best_idx else (1.0, 1.0, 0.0)
            arr.markers.append(self._line_marker(
                "ego_candidate_splines", idx, rgb, path, stamp))
        self.ego_candidate_splines_pub.publish(arr)

    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = OpponentIntentPredictor()
    executor = MultiThreadedExecutor(num_threads=2)
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