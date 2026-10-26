#!/usr/bin/env python3


import math
import random
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from scipy.interpolate import PchipInterpolator
from scipy.spatial import KDTree


@dataclass
class Prediction:
    """Predicted ego state and a candidate merge goal, all in the map frame."""

    goal: np.ndarray
    ego_pred: np.ndarray
    t_pred: float
    ego_yaw_pred: float


class RRTNode:
    __slots__ = ("x", "y", "parent", "cost")

    def __init__(self, x: float, y: float, parent=None, cost: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.parent = parent
        self.cost = float(cost)

    def point(self) -> np.ndarray:
        return np.array((self.x, self.y), dtype=float)


class RRTStarPlanner:
    """Small, bounded-time RRT* implementation used only on live sensor data."""

    def __init__(
        self,
        obstacle_tree: KDTree,
        bounds: Sequence[float],
        *,
        expand_distance: float = 0.45,
        collision_resolution: float = 0.06,
        clearance: float = 0.38,
        max_iterations: int = 750,
        time_budget_s: float = 0.045,
        guide_points: Optional[Sequence[Sequence[float]]] = None,
        guide_probability: float = 0.35,
    ):
        self.tree = obstacle_tree
        self.min_x, self.max_x, self.min_y, self.max_y = map(float, bounds)
        self.expand_distance = float(expand_distance)
        self.collision_resolution = float(collision_resolution)
        self.clearance = float(clearance)
        self.max_iterations = int(max_iterations)
        self.time_budget_s = float(time_budget_s)
        self.guide_points = [np.asarray(p, dtype=float) for p in (guide_points or [])]
        self.guide_probability = float(guide_probability)
        self.nodes: List[RRTNode] = []
        self._xy = np.empty((0, 2), dtype=float)
        self.goal: Optional[RRTNode] = None

    def _append(self, node: RRTNode) -> None:
        self.nodes.append(node)
        self._xy = np.vstack((self._xy, ((node.x, node.y),)))

    @staticmethod
    def _distance(a: RRTNode, b: RRTNode) -> Tuple[float, float]:
        dx, dy = b.x - a.x, b.y - a.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def _sample(self) -> RRTNode:
        # Goal bias is deliberately low.  Side-guide bias is dynamic and merely
        # makes the live RRT converge faster; it is not a fixed map waypoint.
        r = random.random()
        if self.guide_points and r < self.guide_probability:
            p = random.choice(self.guide_points)
            return RRTNode(p[0] + random.uniform(-0.22, 0.22),
                           p[1] + random.uniform(-0.22, 0.22))
        if self.goal is not None and r > 0.87:
            return RRTNode(self.goal.x, self.goal.y)
        return RRTNode(random.uniform(self.min_x, self.max_x),
                       random.uniform(self.min_y, self.max_y))

    def _nearest_index(self, point: RRTNode) -> int:
        d2 = np.sum((self._xy - (point.x, point.y)) ** 2, axis=1)
        return int(np.argmin(d2))

    def _near_indices(self, point: RRTNode) -> List[int]:
        n = max(2, len(self.nodes))
        # Standard RRT* radius with a practical lower bound so rewiring still
        # happens in a short F1TENTH overtaking corridor.
        radius = max(1.15 * self.expand_distance,
                     min(1.65, 2.2 * self.expand_distance * math.sqrt(math.log(n) / n)))
        d2 = np.sum((self._xy - (point.x, point.y)) ** 2, axis=1)
        return [int(i) for i in np.flatnonzero(d2 <= radius * radius)]

    def _steer(self, source: RRTNode, target: RRTNode) -> RRTNode:
        distance, heading = self._distance(source, target)
        step = min(self.expand_distance, distance)
        return RRTNode(source.x + step * math.cos(heading),
                       source.y + step * math.sin(heading),
                       source, source.cost + step)

    def collision_free(self, a: RRTNode, b: RRTNode) -> bool:
        if self.tree is None:
            return False
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        # A zero-length edge is safe only if its point itself is clear.
        count = max(1, int(math.ceil(length / self.collision_resolution)))
        t = np.linspace(0.0, 1.0, count + 1)
        samples = np.column_stack((a.x + t * dx, a.y + t * dy))
        distances, _ = self.tree.query(samples)
        return bool(np.min(distances) > self.clearance)

    def _choose_parent(self, new: RRTNode, near: Sequence[int]) -> Optional[RRTNode]:
        best_parent = new.parent
        best_cost = new.cost
        for index in near:
            parent = self.nodes[index]
            if not self.collision_free(parent, new):
                continue
            d, _ = self._distance(parent, new)
            candidate_cost = parent.cost + d
            if candidate_cost < best_cost:
                best_parent, best_cost = parent, candidate_cost
        if best_parent is None:
            return None
        new.parent = best_parent
        new.cost = best_cost
        return new

    def _propagate_cost(self, root: RRTNode) -> None:
        # Rewiring a node changes descendants' cost.  This keeps the RRT* cost
        # ordering internally consistent without changing path geometry.
        pending = [root]
        while pending:
            parent = pending.pop()
            for child in self.nodes:
                if child.parent is parent:
                    d, _ = self._distance(parent, child)
                    child.cost = parent.cost + d
                    pending.append(child)

    def _rewire(self, new: RRTNode, near: Sequence[int]) -> None:
        for index in near:
            old = self.nodes[index]
            if old is new.parent:
                continue
            d, _ = self._distance(new, old)
            if new.cost + d + 1e-9 >= old.cost:
                continue
            if not self.collision_free(new, old):
                continue
            old.parent = new
            old.cost = new.cost + d
            self._propagate_cost(old)

    def _best_goal_parent(self) -> Optional[RRTNode]:
        if self.goal is None:
            return None
        d2 = np.sum((self._xy - (self.goal.x, self.goal.y)) ** 2, axis=1)
        candidates = np.flatnonzero(d2 <= self.expand_distance * self.expand_distance)
        best = None
        best_cost = float("inf")
        for index in candidates:
            node = self.nodes[int(index)]
            if not self.collision_free(node, self.goal):
                continue
            d, _ = self._distance(node, self.goal)
            if node.cost + d < best_cost:
                best, best_cost = node, node.cost + d
        return best

    def plan(self, start: Sequence[float], goal: Sequence[float]) -> Optional[List[List[float]]]:
        start_node = RRTNode(start[0], start[1])
        self.goal = RRTNode(goal[0], goal[1])
        if not self.collision_free(start_node, start_node):
            return None
        if not self.collision_free(self.goal, self.goal):
            return None

        self.nodes = []
        self._xy = np.empty((0, 2), dtype=float)
        self._append(start_node)
        deadline = time.monotonic() + self.time_budget_s
        first_solution_iteration = None

        for iteration in range(self.max_iterations):
            # Do not spend an uncontrolled amount of the 20 Hz ROS callback in
            # the planner.  Once feasible, leave a short refinement window.
            if time.monotonic() >= deadline and iteration >= 80:
                break
            if first_solution_iteration is not None and iteration > first_solution_iteration + 100:
                break

            sampled = self._sample()
            nearest = self.nodes[self._nearest_index(sampled)]
            new = self._steer(nearest, sampled)
            if not self.collision_free(nearest, new):
                continue

            near = self._near_indices(new)
            new = self._choose_parent(new, near)
            if new is None:
                continue
            self._append(new)
            self._rewire(new, near)

            if first_solution_iteration is None and iteration % 12 == 0:
                if self._best_goal_parent() is not None:
                    first_solution_iteration = iteration

        parent = self._best_goal_parent()
        if parent is None:
            return None
        self.goal.parent = parent
        d, _ = self._distance(parent, self.goal)
        self.goal.cost = parent.cost + d

        path: List[List[float]] = [[self.goal.x, self.goal.y]]
        node: Optional[RRTNode] = self.goal
        while node is not None:
            if node is not self.goal:
                path.append([node.x, node.y])
            node = node.parent
        path.reverse()
        return self.shortcut(path)

    def shortcut(self, path: Sequence[Sequence[float]]) -> List[List[float]]:
        if len(path) <= 2:
            return [list(map(float, p)) for p in path]
        result = [list(map(float, path[0]))]
        i = 0
        while i < len(path) - 1:
            chosen = i + 1
            for j in range(len(path) - 1, i, -1):
                if self.collision_free(RRTNode(*path[i]), RRTNode(*path[j])):
                    chosen = j
                    break
            result.append(list(map(float, path[chosen])))
            i = chosen
        return result


class OvertakeInterceptorNode(Node):
    """Produces a safe map-frame overtaking path for `/opp_racecar`."""

    def __init__(self):
        super().__init__("overtake_interceptor")

        # ROS interface kept compatible with the original node.
        self.ego_odom_sub = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.ego_odom_callback, 10
        )
        self.opp_odom_sub = self.create_subscription(
            Odometry, "/opp_racecar/odom", self.opp_odom_callback, 10
        )
        self.scan_sub = self.create_subscription(LaserScan, "/opp_scan", self.scan_callback, 10)
        self.status_sub = self.create_subscription(String, "/overtake_status", self.status_callback, 10)

        self.overtake_path_pub = self.create_publisher(Path, "/overtake_spline", 10)
        self.mode_pub = self.create_publisher(String, "/driving_mode", 10)
        self.marker_pub = self.create_publisher(Marker, "/overtake_marker", 10)

        # Current measured state.
        self.ego_x = self.ego_y = self.ego_yaw = self.ego_speed = 0.0
        self.opp_x = self.opp_y = self.opp_yaw = self.opp_speed = 0.0
        self.ego_vel = np.zeros(2, dtype=float)
        self.opp_vel = np.zeros(2, dtype=float)
        self.ego_yaw_rate = self.opp_yaw_rate = 0.0
        self._previous_state = {"ego": None, "opp": None}
        self.ego_data_received = False
        self.opp_data_received = False

        # Scan state.  The active spline is considered unsafe if this scan is
        # stale: continuing to follow a path using old obstacle data is unsafe.
        self.scan_ranges = np.empty(0, dtype=float)
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0
        self.scan_range_min = 0.05
        
        # EXTENDED LIDAR RANGE to allow planning from further back
        self.scan_range_max = 10.0
        self.lidar_range_cutoff = 10.0
        
        self.scan_received_at = -float("inf")
        self.max_scan_age = 0.30

        # Vehicle / obstacle geometry.  These are physical safety parameters,
        # not map-specific endpoint offsets or static waypoints.
        self.car_length = 0.58
        self.car_width = 0.30
        self.ego_safety_margin = 0.12
        self.body_outline_resolution = 0.08
        self.rrt_clearance = 0.38      # conservative clearance during RRT generation
        self.validation_clearance = 0.38
        self.validate_resolution = 0.045
        # Active-path cancellation is intentionally narrower than planning
        # clearance: only an observed obstacle intersecting the actual spline
        # should delete a committed path, not a predicted/nearby obstacle.
        self.active_intersection_resolution = 0.02
        self.active_intersection_tolerance = 0.04

        # Dynamic prediction and merge settings.
        self.velocity_filter_alpha = 0.45
        self.yaw_rate_filter_alpha = 0.50
        self.max_agent_speed = 12.0
        self.max_yaw_rate = 8.0
        self.max_curvature = 1.0
        self.min_odom_dt = 1e-3
        self.max_odom_dt = 0.5
        self.min_opp_plan_speed = 0.65
        
        # EXTENDED PREDICTION HORIZONS to support generating up to 4.0m behind
        self.intercept_horizon = 3.0
        self.max_goal_reach = 10.0
        
        self.min_merge_lead = 1.70
        self.merge_lead_time = 1.0
        self.endpoint_pass_margin = 1.20
        self.lateral_clearance = 0.85
        self.goal_clearance_margin = 0.04
        self.overtake_distance = 6.0

        # Swept ego corridor used conservatively during generation only.
        # Active-path cancellation uses ego's current body only (see below).
        self.min_sweep_horizon = 0.75
        self.max_sweep_horizon = 4.0
        self.sweep_margin_time = 0.8
        self.sweep_time_step = 0.15
        self.max_sweep_poses = 30

        self.occluder_step = 0.35
        self.occluder_span = 16.0
        self.blind_ring_resolution = 0.05
        self.ego_return_radius = 0.30

        # Planning / handoff settings.
        self.min_engage_distance = 0.10
        self.max_engage_distance = 4.00
        self.min_target_lead_to_plan = 0.10
        
        # Once a safe RRT* spline is accepted, it is a committed maneuver rather
        # than a continuously moving target.  Replanning due only to normal goal
        # drift makes a random RRT* path visibly jump, so it is disabled below.
        self.max_switch_shift = 0.70
        self.current_side = "left"

        # Publish the Path on a state change only.  Some Pure Pursuit nodes reset
        # their nearest-index state every time they receive a Path; publishing an
        # identical message at 20 Hz therefore looks like path flicker.
        self.path_heartbeat_period = 0.0   # 0 = event-driven only
        self.marker_refresh_period = 0.20

        # Failed-plan retry only.  Active cancellation itself is based only on
        # a literal current obstacle/spline intersection.
        self.failed_plan_retry_period = 0.30

        # Completion and re-arm state.  Completion has *no debounce*.
        self.pass_complete_margin = 0.30
        self.pass_rearm_margin = 1.20
        self.rearm_cooldown = 2.5
        self.finish_remaining_distance = 0.35
        self.finish_goal_radius = 0.45
        self.finish_cross_track = 0.55
        self.pass_complete = False
        self.rearm_allowed_time = 0.0

        # Path quality settings.  PCHIP is C1 and shape-preserving; these gates
        # reject folded, duplicated, oscillating, or unsafe tails.
        self.output_spacing = 0.10
        # Bumped to accommodate the aggressive lateral swoop from directly behind
        self.max_spline_curvature = 1.65
        self.max_corner_turn = math.radians(70.0)
        self.min_terminal_alignment = 0.20
        self.tail_probe_length = 0.90
        # Pure Pursuit lead-in bounds (used only for RRT* fallback, not primary Bezier)
        self.start_leadin_min = 1.95
        self.start_leadin_max = 3.20
        self.start_leadin_speed_gain = 0.15
        self.start_leadin_join_min = 1.95

        # Active-path state.  `active_path` is authoritative.  It is cleared
        # before publishing FTG on every termination/collision path.
        self.active_path: Optional[List[List[float]]] = None
        self.active_goal: Optional[np.ndarray] = None
        self.active_prediction: Optional[Prediction] = None
        self.active_created_at = 0.0
        self.last_plan_time = -float("inf")
        self.next_plan_attempt_at = 0.0
        self.last_path_publish_at = -float("inf")
        self.last_marker_publish_at = -float("inf")
        self.last_reject = ""
        self.current_mode = "FTG"
        self.controller_state = ""
        self.tick_count = 0

        self.create_timer(0.05, self.continuous_spline_update)
        self._log(
            "info",
            "Dynamic Overtake planner ready: Hybrid Aggressive Bézier + RRT* fallback enabled",
        )

    # ------------------------------------------------------------------ time/log
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _log(self, level: str, message: str) -> None:
        """Never let a ROS Foxy logging quirk kill the planner callback.

        Foxy's `RcutilsLogger` can retain an active logging context and raise
        ``Logger severity cannot be changed between calls`` when the same node
        alternates info/warn/error calls.  Every planner message therefore uses
        one ROS severity (INFO) and carries its requested severity in text.  If
        logging itself is unavailable, plain stdout is still safe.
        """
        rendered = f"[{str(level).upper()}] {message}"
        try:
            self.get_logger().info(rendered)
        except Exception:
            try:
                print(f"[overtake_interceptor] {rendered}", flush=True)
            except Exception:
                pass

    def _throttled(self, level: str, message: str, every: int = 20) -> None:
        if self.tick_count % every == 0:
            self._log(level, message)

    # ------------------------------------------------------------- ROS callbacks
    @staticmethod
    def quat_to_yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def status_callback(self, msg: String) -> None:
        self.controller_state = msg.data.strip()

    def set_mode(self, mode: str) -> None:
        if mode != self.current_mode:
            self.current_mode = mode
            self.mode_pub.publish(String(data=mode))
            self._log("info", f"Planner mode -> {mode}")

    def _update_agent_velocity(self, agent: str, stamp: float, x: float, y: float, yaw: float, twist) -> None:
        if not math.isfinite(stamp) or stamp <= 0.0:
            stamp = self.now_s()
        previous = self._previous_state[agent]
        measured_velocity = None
        measured_yaw_rate = None

        if previous is not None:
            dt = stamp - previous[0]
            if self.min_odom_dt < dt < self.max_odom_dt:
                vx = (x - previous[1]) / dt
                vy = (y - previous[2]) / dt
                if math.hypot(vx, vy) <= self.max_agent_speed:
                    measured_velocity = np.array((vx, vy), dtype=float)
                dyaw = math.atan2(math.sin(yaw - previous[3]), math.cos(yaw - previous[3]))
                rate = dyaw / dt
                if abs(rate) <= self.max_yaw_rate:
                    measured_yaw_rate = rate

        if measured_velocity is None:
            c, s = math.cos(yaw), math.sin(yaw)
            measured_velocity = np.array(
                (c * twist.linear.x - s * twist.linear.y, s * twist.linear.x + c * twist.linear.y),
                dtype=float,
            )
            measured_yaw_rate = float(twist.angular.z)

        self._previous_state[agent] = (stamp, x, y, yaw)
        a = self.velocity_filter_alpha
        b = self.yaw_rate_filter_alpha
        if agent == "ego":
            self.ego_vel = (1.0 - a) * self.ego_vel + a * measured_velocity
            if measured_yaw_rate is not None:
                self.ego_yaw_rate = (1.0 - b) * self.ego_yaw_rate + b * measured_yaw_rate
        else:
            self.opp_vel = (1.0 - a) * self.opp_vel + a * measured_velocity
            if measured_yaw_rate is not None:
                self.opp_yaw_rate = (1.0 - b) * self.opp_yaw_rate + b * measured_yaw_rate

    def ego_odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.ego_x, self.ego_y = pose.position.x, pose.position.y
        self.ego_yaw = self.quat_to_yaw(pose.orientation)
        self._update_agent_velocity("ego", stamp, self.ego_x, self.ego_y, self.ego_yaw, msg.twist.twist)
        self.ego_speed = float(np.linalg.norm(self.ego_vel))
        self.ego_data_received = True

    def opp_odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.opp_x, self.opp_y = pose.position.x, pose.position.y
        self.opp_yaw = self.quat_to_yaw(pose.orientation)
        self._update_agent_velocity("opp", stamp, self.opp_x, self.opp_y, self.opp_yaw, msg.twist.twist)
        self.opp_speed = float(np.linalg.norm(self.opp_vel))
        self.opp_data_received = True

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_ranges = np.asarray(msg.ranges, dtype=float)
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_increment = float(msg.angle_increment)
        self.scan_range_min = max(0.03, float(getattr(msg, "range_min", 0.05)))
        reported_max = float(getattr(msg, "range_max", self.lidar_range_cutoff))
        self.scan_range_max = reported_max if math.isfinite(reported_max) and reported_max > 0.0 else self.lidar_range_cutoff
        # Freshness is based on local receipt time, not the message stamp.  This
        # remains correct whether the node is using wall time or `/clock`.
        self.scan_received_at = self.now_s()

    # -------------------------------------------------------------- basic geometry
    @staticmethod
    def _unit_from_yaw(yaw: float) -> np.ndarray:
        return np.array((math.cos(yaw), math.sin(yaw)), dtype=float)

    @staticmethod
    def _cross(a: np.ndarray, b: np.ndarray) -> float:
        return float(a[0] * b[1] - a[1] * b[0])

    @staticmethod
    def point_to_rect_distance(
        px: float, py: float, cx: float, cy: float, yaw: float, half_length: float, half_width: float
    ) -> float:
        dx, dy = px - cx, py - cy
        c, s = math.cos(yaw), math.sin(yaw)
        local_x = dx * c + dy * s
        local_y = -dx * s + dy * c
        return math.hypot(max(abs(local_x) - half_length, 0.0), max(abs(local_y) - half_width, 0.0))

    @staticmethod
    def rectangle_outline(
        cx: float, cy: float, yaw: float, half_length: float, half_width: float, resolution: float
    ) -> List[List[float]]:
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(-half_length, -half_width), (half_length, -half_width),
                   (half_length, half_width), (-half_length, half_width)]
        points: List[List[float]] = []
        for i, (x0, y0) in enumerate(corners):
            x1, y1 = corners[(i + 1) % 4]
            count = max(2, int(math.hypot(x1 - x0, y1 - y0) / resolution) + 1)
            for j in range(count):
                t = j / (count - 1)
                lx, ly = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
                points.append([cx + lx * c - ly * s, cy + lx * s + ly * c])
        return points

    @staticmethod
    def dedupe(points: Iterable[Sequence[float]], tolerance: float = 1e-4) -> List[List[float]]:
        result: List[List[float]] = []
        for p in points:
            q = [float(p[0]), float(p[1])]
            if not np.isfinite(q).all():
                continue
            if not result or math.hypot(q[0] - result[-1][0], q[1] - result[-1][1]) > tolerance:
                result.append(q)
        return result

    @staticmethod
    def path_length(path: Optional[Sequence[Sequence[float]]]) -> float:
        if path is None or len(path) < 2:
            return 0.0
        p = np.asarray(path, dtype=float)
        return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))

    @staticmethod
    def _resample_polyline(path: Sequence[Sequence[float]], spacing: float) -> List[List[float]]:
        points = np.asarray(path, dtype=float)
        if len(points) < 2:
            return points.tolist()
        segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
        stations = np.concatenate(([0.0], np.cumsum(segment)))
        if stations[-1] <= 1e-9:
            return [points[0].tolist()]
        # Keep a true fixed arc-length interval.  The supplied Pure Pursuit
        # controller converts metres to indices by dividing by 0.1, so using
        # `linspace` here (which subtly changes every interval) causes it to
        # target the wrong physical lookahead distance.
        spacing = max(float(spacing), 1e-3)
        query = np.arange(0.0, stations[-1], spacing)
        if len(query) == 0 or stations[-1] - query[-1] > 1e-6:
            query = np.append(query, stations[-1])
        if len(query) == 1:
            query = np.append(query, stations[-1])
        return np.column_stack((np.interp(query, stations, points[:, 0]),
                                np.interp(query, stations, points[:, 1]))).tolist()

    # ----------------------------------------------------- motion / pass semantics
    @staticmethod
    def _predict_pose(x: float, y: float, yaw: float, yaw_rate: float, speed: float, t: float) -> Tuple[float, float, float]:
        if abs(yaw_rate) < 1e-3 or speed < 1e-3:
            return x + speed * t * math.cos(yaw), y + speed * t * math.sin(yaw), yaw + yaw_rate * t
        radius = speed / yaw_rate
        future_yaw = yaw + yaw_rate * t
        return (x + radius * (math.sin(future_yaw) - math.sin(yaw)),
                y - radius * (math.cos(future_yaw) - math.cos(yaw)), future_yaw)

    def ego_motion_heading(self) -> float:
        if float(np.linalg.norm(self.ego_vel)) >= 0.20:
            return math.atan2(float(self.ego_vel[1]), float(self.ego_vel[0]))
        return self.ego_yaw

    def ego_pose_at(self, t: float) -> Tuple[float, float, float]:
        speed = float(np.linalg.norm(self.ego_vel))
        rate = float(np.clip(self.ego_yaw_rate, -self.max_yaw_rate, self.max_yaw_rate))
        if speed > 0.25:
            rate = float(np.clip(rate, -self.max_curvature * speed, self.max_curvature * speed))
        else:
            rate = 0.0
        return self._predict_pose(self.ego_x, self.ego_y, self.ego_motion_heading(), rate, speed, max(0.0, t))

    def ego_track_axis(self) -> np.ndarray:
        """Local track direction inferred from the moving target, never a map waypoint."""
        return self._unit_from_yaw(self.ego_motion_heading())

    def longitudinal_to_ego(self) -> float:
        """Positive means the commanded opponent is ahead of ego along live track direction."""
        return float(np.dot(np.array((self.opp_x - self.ego_x, self.opp_y - self.ego_y)), self.ego_track_axis()))

    def opponent_is_ahead(self) -> bool:
        return self.longitudinal_to_ego() >= self.pass_complete_margin

    def ego_is_ahead_enough_to_plan(self) -> bool:
        return self.longitudinal_to_ego() <= -self.min_target_lead_to_plan

    @staticmethod
    def _solve_intercept_time(relative_target: np.ndarray, target_velocity: np.ndarray, pursuer_speed: float, t_max: float) -> float:
        # Solve |relative_target + target_velocity*t| = pursuer_speed*t.
        a = float(target_velocity @ target_velocity) - pursuer_speed * pursuer_speed
        b = 2.0 * float(relative_target @ target_velocity)
        c = float(relative_target @ relative_target)
        roots: List[float] = []
        if abs(a) < 1e-6:
            if abs(b) > 1e-9:
                roots.append(-c / b)
        else:
            discriminant = b * b - 4.0 * a * c
            if discriminant >= 0.0:
                root = math.sqrt(discriminant)
                roots.extend(((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)))
        positive = [r for r in roots if r > 0.0 and math.isfinite(r)]
        return float(min(min(positive), t_max)) if positive else float(t_max)

    def compute_predicted_goal(self, side: str, t_override: Optional[float] = None) -> Prediction:
        """Predict a moving, curved ego trajectory and place a merge point ahead of it."""
        opp = np.array((self.opp_x, self.opp_y), dtype=float)
        ego = np.array((self.ego_x, self.ego_y), dtype=float)
        opp_speed = max(float(np.linalg.norm(self.opp_vel)), self.min_opp_plan_speed)

        if t_override is None:
            t_pred = self._solve_intercept_time(ego - opp, self.ego_vel, opp_speed, self.intercept_horizon)
            # Refine against the curved ego prediction rather than a static line.
            for _ in range(3):
                px, py, _ = self.ego_pose_at(t_pred)
                revised = min(float(math.hypot(px - self.opp_x, py - self.opp_y)) / opp_speed,
                              self.intercept_horizon)
                if abs(revised - t_pred) < 0.02:
                    break
                t_pred = 0.5 * (t_pred + revised)
        else:
            t_pred = float(np.clip(t_override, 0.05, self.intercept_horizon + 1.0))

        ex, ey, predicted_yaw = self.ego_pose_at(t_pred)
        predicted_ego = np.array((ex, ey), dtype=float)
        ego_speed = max(float(np.linalg.norm(self.ego_vel)), self.min_opp_plan_speed)
        lead = max(self.min_merge_lead, ego_speed * self.merge_lead_time, self.endpoint_pass_margin + 0.25)
        yaw_rate = float(np.clip(self.ego_yaw_rate, -self.max_curvature * ego_speed, self.max_curvature * ego_speed))
        gx, gy, goal_yaw = self._predict_pose(ex, ey, predicted_yaw, yaw_rate, ego_speed, lead / ego_speed)
        forward = self._unit_from_yaw(goal_yaw)
        lateral = np.array((-forward[1], forward[0])) if side == "left" else np.array((forward[1], -forward[0]))
        goal = np.array((gx, gy), dtype=float) + self.lateral_clearance * lateral
        return Prediction(goal=goal, ego_pred=predicted_ego, t_pred=t_pred, ego_yaw_pred=goal_yaw)

    def endpoint_overtakes_ego(self, endpoint: Sequence[float], prediction: Prediction, *, strict: bool = True) -> bool:
        endpoint = np.asarray(endpoint, dtype=float)
        predicted_forward = self._unit_from_yaw(prediction.ego_yaw_pred)
        lead_at_arrival = float(np.dot(endpoint - prediction.ego_pred, predicted_forward))
        current_lead = float(np.dot(endpoint - np.array((self.ego_x, self.ego_y)), self.ego_track_axis()))
        required = self.endpoint_pass_margin if strict else 0.55 * self.endpoint_pass_margin
        # The first term is the key guarantee: the physical end point is ahead
        # of where ego will be at arrival, not merely alongside ego now.
        return lead_at_arrival >= required and current_lead >= 0.20

    # ---------------------------------------------------------- target swept body
    def sweep_horizon(self, base_time: float) -> float:
        return float(np.clip(base_time + self.sweep_margin_time, self.min_sweep_horizon, self.max_sweep_horizon))

    def ego_corridor_points(self, horizon: float) -> List[List[float]]:
        count = min(self.max_sweep_poses, max(2, int(math.ceil(horizon / self.sweep_time_step)) + 1))
        times = np.linspace(0.0, horizon, count)
        half_length = self.car_length / 2.0 + self.ego_safety_margin
        half_width = self.car_width / 2.0 + self.ego_safety_margin
        points: List[List[float]] = []
        for t in times:
            x, y, yaw = self.ego_pose_at(float(t))
            points.extend(self.rectangle_outline(x, y, yaw, half_length, half_width, self.body_outline_resolution))
        return points

    # ------------------------------------------------------------ live obstacles
    def scan_is_fresh(self) -> bool:
        return len(self.scan_ranges) > 0 and self.now_s() - self.scan_received_at <= self.max_scan_age

    def lidar_environment_points(self) -> Optional[List[List[float]]]:
        """Return conservative world-frame wall/unknown-space points from `/opp_scan`."""
        if not self.scan_is_fresh():
            return None
        n = len(self.scan_ranges)
        if n == 0 or abs(self.scan_angle_increment) < 1e-9:
            return None

        ranges = self.scan_ranges
        cutoff = min(self.lidar_range_cutoff, self.scan_range_max)
        r_min = max(self.scan_range_min, 0.03)
        finite_return = np.isfinite(ranges) & (ranges > r_min) & (ranges < cutoff)
        points: List[List[float]] = []
        ego_half_length, ego_half_width = self.car_length / 2.0, self.car_width / 2.0

        for i in range(n):
            angle = self.opp_yaw + self.scan_angle_min + i * self.scan_angle_increment
            c, s = math.cos(angle), math.sin(angle)
            if finite_return[i]:
                r = float(ranges[i])
                px, py = self.opp_x + r * c, self.opp_y + r * s
                # A lidar return from ego is represented by the much more
                # accurate moving rectangular corridor below, not as a wall.
                ego_return = self.point_to_rect_distance(
                    px, py, self.ego_x, self.ego_y, self.ego_yaw, ego_half_length, ego_half_width
                ) <= self.ego_return_radius
                if ego_return:
                    continue
                points.append([px, py])
                # Opaque wall: do not let a segment step through the thin return
                # and emerge in unknown space behind it.
                r_band = r + self.occluder_step
                while r_band <= min(cutoff, r + self.occluder_span):
                    points.append([self.opp_x + r_band * c, self.opp_y + r_band * s])
                    r_band += self.occluder_step
            else:
                # No return is a sensor-horizon boundary, not free map space.
                points.append([self.opp_x + cutoff * c, self.opp_y + cutoff * s])

        # A typical F1TENTH lidar does not cover 360 degrees.  Close the unseen
        # sector with a horizon ring so the RRT cannot route behind the car into
        # completely unobserved space.
        covered = abs((n - 1) * self.scan_angle_increment)
        if covered < 2.0 * math.pi - 0.02:
            start = self.scan_angle_min
            end = self.scan_angle_min + (n - 1) * self.scan_angle_increment
            low, high = min(start, end), max(start, end)
            blind = 2.0 * math.pi - (high - low)
            count = max(4, int(math.ceil(blind / self.blind_ring_resolution)))
            for j in range(count + 1):
                a = high + blind * j / count
                angle = self.opp_yaw + a
                points.append([self.opp_x + cutoff * math.cos(angle), self.opp_y + cutoff * math.sin(angle)])
        return points

    def current_observed_obstacle_points(self) -> Optional[List[List[float]]]:
        """Actual lidar returns at this instant, with no predicted/virtual walls.

        This is deliberately separate from `lidar_environment_points()`.  The
        latter includes occlusion bands and an unknown-space horizon, which are
        appropriate while *planning* but are not physical intersections and must
        not make an already committed spline disappear.
        """
        if not self.scan_is_fresh():
            return None
        n = len(self.scan_ranges)
        if n == 0 or abs(self.scan_angle_increment) < 1e-9:
            return None
        cutoff = min(self.lidar_range_cutoff, self.scan_range_max)
        r_min = max(self.scan_range_min, 0.03)
        ranges = self.scan_ranges
        valid = np.isfinite(ranges) & (ranges > r_min) & (ranges < cutoff)
        points: List[List[float]] = []
        ego_half_length, ego_half_width = self.car_length / 2.0, self.car_width / 2.0
        for i in np.flatnonzero(valid):
            angle = self.opp_yaw + self.scan_angle_min + int(i) * self.scan_angle_increment
            r = float(ranges[int(i)])
            px, py = self.opp_x + r * math.cos(angle), self.opp_y + r * math.sin(angle)
            # Ego is checked exactly as a current rectangle below, rather than
            # turning its lidar return into a point obstacle here.
            if self.point_to_rect_distance(px, py, self.ego_x, self.ego_y,
                                           self.ego_yaw, ego_half_length, ego_half_width) <= 0.0:
                continue
            points.append([px, py])
        return points

    def barrier_tree(self, horizon: float) -> Optional[KDTree]:
        environment = self.lidar_environment_points()
        if environment is None:
            return None
        all_points = environment + self.ego_corridor_points(horizon)
        if not all_points:
            return None
        return KDTree(np.asarray(all_points, dtype=float))

    def _sample_path(self, path: Sequence[Sequence[float]], resolution: Optional[float] = None) -> np.ndarray:
        if path is None or len(path) < 2:
            return np.empty((0, 2), dtype=float)
        resolution = self.validate_resolution if resolution is None else resolution
        points = np.asarray(path, dtype=float)
        output = [points[0]]
        for a, b in zip(points[:-1], points[1:]):
            length = float(np.linalg.norm(b - a))
            count = max(1, int(math.ceil(length / resolution)))
            t = np.linspace(0.0, 1.0, count + 1)[1:]
            output.extend(a + (b - a) * value for value in t)
        return np.asarray(output, dtype=float)

    def path_min_clearance(self, path: Sequence[Sequence[float]], tree: Optional[KDTree]) -> float:
        """Dense minimum clearance for a path; invalid data is unsafe."""
        if tree is None or path is None or len(path) < 2:
            return -float("inf")
        samples = self._sample_path(path)
        if len(samples) == 0 or not np.isfinite(samples).all():
            return -float("inf")
        distance, _ = tree.query(samples)
        return float(np.min(distance))

    def path_is_clear(self, path: Sequence[Sequence[float]], tree: Optional[KDTree]) -> bool:
        return self.path_min_clearance(path, tree) > self.validation_clearance

    # --------------------------------------------------------------- path progress
    @staticmethod
    def project_to_polyline(point: Sequence[float], path: Sequence[Sequence[float]]) -> Tuple[float, float, np.ndarray, int, float]:
        """Return station, cross-track distance, closest point, segment index, total length."""
        points = np.asarray(path, dtype=float)
        if len(points) < 2:
            return 0.0, float("inf"), points[0] if len(points) else np.zeros(2), 0, 0.0
        segment = np.diff(points, axis=0)
        lengths = np.linalg.norm(segment, axis=1)
        stations = np.concatenate(([0.0], np.cumsum(lengths)))
        best_distance = float("inf")
        best_station = 0.0
        best_point = points[0]
        best_index = 0
        query = np.asarray(point, dtype=float)
        for i, (a, vec, length) in enumerate(zip(points[:-1], segment, lengths)):
            if length < 1e-9:
                continue
            t = float(np.clip(np.dot(query - a, vec) / (length * length), 0.0, 1.0))
            candidate = a + t * vec
            distance = float(np.linalg.norm(query - candidate))
            if distance < best_distance:
                best_distance = distance
                best_station = float(stations[i] + t * length)
                best_point = candidate
                best_index = i
        return best_station, best_distance, best_point, best_index, float(stations[-1])

    def remaining_active_path(self) -> Optional[List[List[float]]]:
        if self.active_path is None or len(self.active_path) < 2:
            return None
        car = np.array((self.opp_x, self.opp_y), dtype=float)
        _, _, closest, index, _ = self.project_to_polyline(car, self.active_path)
        remaining: List[List[float]] = [car.tolist()]
        if float(np.linalg.norm(closest - car)) > 1e-4:
            remaining.append(closest.tolist())
        remaining.extend(self.active_path[index + 1:])
        result = self.dedupe(remaining)
        return result if len(result) >= 2 else None

    def active_completion_reason(self) -> Optional[str]:
        """Detect completion on this tick; callers must clear immediately."""
        if self.active_path is None:
            return None
        state = self.controller_state.upper()
        if state in {"PATH_COMPLETE", "SPLINE_COMPLETE", "OVERTAKE_COMPLETE", "FINISHED"}:
            return f"Controller reported {state}"
        if self.opponent_is_ahead():
            return f"Opponent passed ego ({self.longitudinal_to_ego():.2f} m ahead)"

        station, cross_track, _, _, total = self.project_to_polyline((self.opp_x, self.opp_y), self.active_path)
        remaining = max(0.0, total - station)
        endpoint = np.asarray(self.active_path[-1], dtype=float)
        end_distance = float(np.linalg.norm(endpoint - np.array((self.opp_x, self.opp_y))))
        # A completion needs both terminal progress and proximity.  This avoids
        # falsely consuming a path merely because two distant curve sections are
        # spatially close, while still deleting it in the same callback at end.
        if (remaining <= self.finish_remaining_distance and
                end_distance <= self.finish_goal_radius and
                cross_track <= self.finish_cross_track):
            return f"Spline consumed (remaining {remaining:.2f} m)"

        opp_forward = self._unit_from_yaw(self.opp_yaw)
        if float(np.dot(endpoint - np.array((self.opp_x, self.opp_y)), opp_forward)) < -0.10:
            return "Opponent has driven beyond spline endpoint"
        return None

    # ---------------------------------------------------------- path quality gate
    @staticmethod
    def _orientation(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        return float((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))

    @classmethod
    def _segments_intersect(cls, a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
        # Proper/interior intersection is enough for rejecting a folded path.
        o1, o2 = cls._orientation(a, b, c), cls._orientation(a, b, d)
        o3, o4 = cls._orientation(c, d, a), cls._orientation(c, d, b)
        return (o1 * o2 < -1e-10) and (o3 * o4 < -1e-10)

    def _self_intersects(self, points: np.ndarray) -> bool:
        """Vectorised non-adjacent segment test (keeps quality gating cheap)."""
        # A direct nested Python loop dominated the old planner callback when it
        # checked 70--100 output samples.  Test all later segments for each first
        # segment in NumPy instead; this preserves the fold/loop rejection while
        # keeping the planner within its real-time budget.
        segment_count = len(points) - 1
        for i in range(segment_count - 2):
            a = points[i]
            b = points[i + 1]
            c = points[i + 2:-1]
            d = points[i + 3:]
            if len(c) == 0:
                continue
            r = b - a
            s = d - c
            ac = c - a
            ad = d - a
            ca = a - c
            cb = b - c
            o1 = r[0] * ac[:, 1] - r[1] * ac[:, 0]
            o2 = r[0] * ad[:, 1] - r[1] * ad[:, 0]
            o3 = s[:, 0] * ca[:, 1] - s[:, 1] * ca[:, 0]
            o4 = s[:, 0] * cb[:, 1] - s[:, 1] * cb[:, 0]
            if np.any((o1 * o2 < -1e-10) & (o3 * o4 < -1e-10)):
                return True
        return False

    def path_has_good_shape(self, path: Sequence[Sequence[float]], prediction: Prediction) -> bool:
        points = np.asarray(path, dtype=float)
        if len(points) < 3 or not np.isfinite(points).all():
            return False
        vectors = np.diff(points, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        if np.any(lengths < 1e-4):
            return False
        directions = vectors / lengths[:, None]
        if self._self_intersects(points):
            return False
        if len(directions) > 1:
            dot = np.clip(np.sum(directions[:-1] * directions[1:], axis=1), -1.0, 1.0)
            turns = np.arccos(dot)
            if float(np.max(turns)) > self.max_corner_turn:
                return False
            if self.path_max_curvature(points) > self.max_spline_curvature:
                return False

        # The first Pure Pursuit target must be forward along the commanded
        # vehicle heading.  A side/backwards first segment is rejected even when
        # it is geometrically collision-free.
        start_forward = self._unit_from_yaw(self.opp_yaw)
        entry_count = min(len(directions), 6)
        if float(np.dot(directions[0], start_forward)) < 0.995:
            return False
        # The clamped entry curve may begin turning gently, but it may never
        # point sideways/backwards during the first Pure Pursuit lookahead.
        if np.any(directions[:entry_count] @ start_forward < 0.55):
            return False

        target_forward = self._unit_from_yaw(prediction.ego_yaw_pred)
        # The end of the curve must merge forward, never turn back toward ego or
        # fold into a wall after the final control point.
        tail_count = min(5, len(directions))
        if float(np.dot(directions[-1], target_forward)) < self.min_terminal_alignment:
            return False
        if np.any(directions[-tail_count:] @ target_forward < -0.05):
            return False
        return True

    def terminal_leadout_clearance(self, path: Sequence[Sequence[float]], tree: KDTree, prediction: Prediction) -> float:
        """Return clearance along the terminal tangent, or -inf for an invalid tail."""
        points = np.asarray(path, dtype=float)
        if len(points) < 2:
            return -float("inf")
        tangent = points[-1] - points[-2]
        norm = float(np.linalg.norm(tangent))
        if norm < 1e-5:
            return -float("inf")
        tangent /= norm
        target_forward = self._unit_from_yaw(prediction.ego_yaw_pred)
        if float(np.dot(tangent, target_forward)) < self.min_terminal_alignment:
            return -float("inf")
        distance = np.linspace(0.08, self.tail_probe_length, max(4, int(self.tail_probe_length / 0.08)))
        probe = points[-1] + distance[:, None] * tangent
        clearance, _ = tree.query(probe)
        return float(np.min(clearance))

    def terminal_leadout_is_safe(self, path: Sequence[Sequence[float]], tree: KDTree, prediction: Prediction) -> bool:
        return self.terminal_leadout_clearance(path, tree, prediction) > self.validation_clearance

    def candidate_is_valid(self, path: Sequence[Sequence[float]], tree: KDTree, prediction: Prediction) -> bool:
        path = self.dedupe(path)
        if len(path) < 3:
            return False
        if not self.endpoint_overtakes_ego(path[-1], prediction, strict=True):
            self.last_reject = "terminal point does not overtake predicted ego"
            return False
        if not self.path_has_good_shape(path, prediction):
            self.last_reject = "spline has a cusp, fold, loop, or bad terminal tangent"
            return False
        if not self.path_is_clear(path, tree):
            self.last_reject = "spline intersects lidar wall/ego corridor"
            return False
        if not self.terminal_leadout_is_safe(path, tree, prediction):
            self.last_reject = "spline tail points into an obstacle"
            return False
        return True

    # ---------------------------------------------------------- RRT path shaping
    def _add_heading_guides(
        self,
        path: Sequence[Sequence[float]],
        goal: np.ndarray,
        prediction: Prediction,
        side: str,
        planner: RRTStarPlanner,
    ) -> List[List[float]]:
        result = self.dedupe(path)
        if len(result) < 2:
            return result

        # Insert a *dynamic* lane-change ramp only when all of its segments are
        # RRT collision-free.  The ramp scales with vehicle clearance rather than
        # relying on a Spielberg-specific waypoint.  It prevents a valid RRT
        # polyline from asking pure pursuit to make an abrupt 90-degree turn at
        # the start of an otherwise good pass.
        opp_forward = self._unit_from_yaw(self.opp_yaw)
        start = np.array((self.opp_x, self.opp_y), dtype=float)
        start_lateral = np.array((-opp_forward[1], opp_forward[0])) if side == "left" else np.array((opp_forward[1], -opp_forward[0]))
        
        # Scale ramp dynamically so we don't plan straight through ego when close
        dist_to_ego = math.hypot(self.ego_x - self.opp_x, self.ego_y - self.opp_y)
        safe_ramp_limit = max(0.9, dist_to_ego - 0.4)
        ramp_distance = min(max(2.25, 2.50 * self.lateral_clearance), safe_ramp_limit)
        ramp_offset = 0.80 * self.lateral_clearance
        
        # Quintic smoothstep has zero lateral slope and curvature tendency at
        # both ends.  Sampling this curve provides a physically gentle entry,
        # instead of a horizontal segment followed by a sharp RRT corner.
        entry_guides = []
        for fraction in np.linspace(1.0 / 6.0, 1.0, 6):
            smoothstep = 10.0 * fraction**3 - 15.0 * fraction**4 + 6.0 * fraction**5
            entry_guides.append(start + fraction * ramp_distance * opp_forward +
                                smoothstep * ramp_offset * start_lateral)
        chain = [start] + entry_guides + [np.asarray(result[1], dtype=float)]
        if all(planner.collision_free(RRTNode(*a), RRTNode(*b)) for a, b in zip(chain[:-1], chain[1:])):
            result[1:1] = [p.tolist() for p in entry_guides]
        else:
            # A smaller forward tangent is still useful when the full ramp does
            # not fit around a nearby wall or close vehicle.
            safe_start = min(0.55, dist_to_ego * 0.3)
            start_guide = start + safe_start * opp_forward
            if (planner.collision_free(RRTNode(*start), RRTNode(*start_guide)) and
                    planner.collision_free(RRTNode(*start_guide), RRTNode(*result[1]))):
                result.insert(1, start_guide.tolist())

        # A final guide gives the shape-preserving spline a forward merge tangent.
        target_forward = self._unit_from_yaw(prediction.ego_yaw_pred)
        end_guide = goal - 0.80 * target_forward
        if len(result) >= 2 and (
            planner.collision_free(RRTNode(*result[-2]), RRTNode(*end_guide)) and
            planner.collision_free(RRTNode(*end_guide), RRTNode(*goal))
        ):
            result.insert(-1, end_guide.tolist())
        return self.dedupe(result)

    @staticmethod
    def _chaikin(path: Sequence[Sequence[float]], rounds: int = 2) -> List[List[float]]:
        result = [np.asarray(p, dtype=float) for p in path]
        for _ in range(rounds):
            if len(result) < 3:
                break
            next_points = [result[0]]
            for a, b in zip(result[:-1], result[1:]):
                next_points.append(0.75 * a + 0.25 * b)
                next_points.append(0.25 * a + 0.75 * b)
            next_points.append(result[-1])
            result = next_points
        return [p.tolist() for p in result]

    @staticmethod
    def _laplacian_smooth(path: Sequence[Sequence[float]], weight: float, iterations: int) -> List[List[float]]:
        """Low-pass filter a polyline while keeping its start and goal exact."""
        points = np.asarray(path, dtype=float).copy()
        if len(points) < 3:
            return points.tolist()
        weight = float(np.clip(weight, 0.0, 0.49))
        for _ in range(max(0, int(iterations))):
            updated = points.copy()
            updated[1:-1] = points[1:-1] + weight * (
                0.5 * (points[:-2] + points[2:]) - points[1:-1]
            )
            points = updated
        return points.tolist()

    @staticmethod
    def path_max_curvature(path: Sequence[Sequence[float]]) -> float:
        points = np.asarray(path, dtype=float)
        if len(points) < 3:
            return float("inf")
        vector = np.diff(points, axis=0)
        length = np.linalg.norm(vector, axis=1)
        if np.any(length < 1e-6):
            return float("inf")
        direction = vector / length[:, None]
        turn = np.arccos(np.clip(np.sum(direction[:-1] * direction[1:], axis=1), -1.0, 1.0))
        local_length = 0.5 * (length[:-1] + length[1:])
        return float(np.max(turn / np.maximum(local_length, 1e-4))) if len(turn) else 0.0

    @staticmethod
    def _point_tangent_suffix_at_station(
        path: Sequence[Sequence[float]], station: float
    ) -> Tuple[np.ndarray, np.ndarray, List[List[float]]]:
        """Interpolate a point/tangent and return the remaining suffix there."""
        points = np.asarray(path, dtype=float)
        segment = np.diff(points, axis=0)
        lengths = np.linalg.norm(segment, axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        station = float(np.clip(station, 0.0, cumulative[-1]))
        index = min(int(np.searchsorted(cumulative, station, side="right") - 1), len(segment) - 1)
        while index < len(segment) - 1 and lengths[index] < 1e-6:
            index += 1
        if lengths[index] < 1e-6:
            tangent = np.array((1.0, 0.0), dtype=float)
            point = points[index]
        else:
            fraction = float(np.clip((station - cumulative[index]) / lengths[index], 0.0, 1.0))
            point = points[index] + fraction * segment[index]
            tangent = segment[index] / lengths[index]
        return point, tangent, [point.tolist()] + points[index + 1:].tolist()

    def enforce_pure_pursuit_leadin(self, path: Sequence[Sequence[float]]) -> Optional[List[List[float]]]:
        """Clamp the beginning of a candidate to the opponent's current heading.

        RRT* is free to pick a safe first branch at any angle.  That is correct
        geometrically but gives Pure Pursuit an immediate sideways target.  A
        clamped C2 quintic Bezier entry fixes the initial heading and initial
        curvature without using any map-specific waypoint.  The caller performs
        the normal collision and curvature validation on the final result.
        """
        points = np.asarray(path, dtype=float)
        if len(points) < 4:
            return None
        total = self.path_length(points)
        start = np.array((self.opp_x, self.opp_y), dtype=float)
        if float(np.linalg.norm(points[0] - start)) > 0.15:
            # A plan must originate at the current commanded-car pose; do not
            # splice an entry curve onto an already stale path.
            return None

        speed = float(np.linalg.norm(self.opp_vel))
        
        # Do not rigidly enforce a straight path longer than the distance to ego!
        dist_to_ego = math.hypot(self.ego_x - self.opp_x, self.ego_y - self.opp_y)
        max_leadin = max(0.8, dist_to_ego - 0.4) 
        desired_leadin = self.start_leadin_min + self.start_leadin_speed_gain * speed
        
        min_bound = min(self.start_leadin_join_min, max_leadin)
        max_bound = min(self.start_leadin_max, max_leadin)
        join_station = float(np.clip(desired_leadin, min_bound, max_bound))
        
        # Do not consume too much of a short maneuver; it must still have a
        # nontrivial suffix after the smooth entry transition.
        join_station = min(join_station, 0.45 * total)
        if join_station <= 0.45:  # Dropped from 0.70 to allow tighter turns when ego is close
            return None

        join, end_tangent, suffix = self._point_tangent_suffix_at_station(points, join_station)
        start_tangent = self._unit_from_yaw(self.opp_yaw)
        if float(np.dot(end_tangent, start_tangent)) < -0.20:
            return None

        bridge_length = float(np.linalg.norm(join - start))
        if bridge_length < 0.50:
            return None

        # Quintic Bezier controls. P1/P2 and P4/P3 are collinear pairs, which
        # gives zero initial/final curvature while retaining the exact start and
        # suffix tangents. This is much friendlier to Pure Pursuit than forcing a
        # straight segment followed by an abrupt lane-change bend.
        
        # Scale the Bezier arms down if ego is close so the curve can bend sharply enough
        safe_arm = max(0.25, dist_to_ego * 0.35)
        start_arm = min(0.35 * bridge_length, min(0.95, safe_arm))
        end_arm = min(0.30 * bridge_length, 0.80)

        c1 = start + start_arm * start_tangent
        c2 = start + 2.0 * start_arm * start_tangent
        c4 = join - end_arm * end_tangent
        c3 = join - 2.0 * end_arm * end_tangent

        curve_count = max(12, int(math.ceil(bridge_length / self.output_spacing)) + 1)
        leadin: List[List[float]] = []
        for t in np.linspace(0.0, 1.0, curve_count):
            omt = 1.0 - t
            point = (omt ** 5) * start + 5.0 * (omt ** 4) * t * c1 + \
                    10.0 * (omt ** 3) * (t ** 2) * c2 + 10.0 * (omt ** 2) * (t ** 3) * c3 + \
                    5.0 * omt * (t ** 4) * c4 + (t ** 5) * join
            leadin.append(point.tolist())
        leadin.extend(suffix[1:])
        return self.dedupe(leadin)

    def smooth_validated_path(self, raw_path: Sequence[Sequence[float]], tree: KDTree, prediction: Prediction) -> Optional[List[List[float]]]:
        raw = self.dedupe(raw_path)
        if len(raw) < 2:
            return None
        source = np.asarray(raw, dtype=float)
        segment = np.linalg.norm(np.diff(source, axis=0), axis=1)
        stations = np.concatenate(([0.0], np.cumsum(segment)))
        if stations[-1] <= 1e-6:
            return None
        count = max(24, int(math.ceil(stations[-1] / self.output_spacing)) + 1)
        query = np.linspace(0.0, stations[-1], count)
        candidates: List[List[List[float]]] = []

        # First smooth the dense RRT route with a low-pass/elastic-band pass.
        # This reduces short RRT steering oscillations without moving either the
        # commanded car's start or the validated passing endpoint.  Any curve
        # that rounds into an obstacle is rejected below, never published.
        for weight, iterations in ((0.46, 42), (0.40, 28), (0.32, 16)):
            filtered = self._laplacian_smooth(raw, weight, iterations)
            candidates.append(self._resample_polyline(filtered, self.output_spacing))

        # Chaikin converges to a smooth B-spline-like curve while preserving both
        # endpoints.  It is a second independent smoother in case the elastic
        # pass cannot clear a particular RRT corner.
        candidates.append(self._resample_polyline(self._chaikin(raw, rounds=5), self.output_spacing))
        candidates.append(self._resample_polyline(self._chaikin(raw, rounds=4), self.output_spacing))
        candidates.append(self._resample_polyline(self._chaikin(raw, rounds=3), self.output_spacing))

        # PCHIP remains a bounded C1 fallback, but is selected only if it is
        # genuinely lower-curvature than the stronger smoothers above.
        if len(raw) >= 3:
            try:
                spline_x = PchipInterpolator(stations, source[:, 0])
                spline_y = PchipInterpolator(stations, source[:, 1])
                pchip = np.column_stack((spline_x(query), spline_y(query)))
                pchip[0] = source[0]
                pchip[-1] = source[-1]
                candidates.append(self.dedupe(pchip.tolist()))
            except Exception as exc:
                self._throttled("warning", f"PCHIP smoothing failed: {exc}", every=40)

        # Deliberately no raw-polyline fallback: a sharp RRT polyline may be
        # collision-free yet still be impossible for Pure Pursuit to follow.
        # Candidates are ordered from strongest/lowest-curvature smoothing to
        # gentlest smoothing.  Return the first safe one so normal planning stays
        # real-time; candidates that cut into an obstacle are skipped.
        for candidate in candidates:
            candidate = self.dedupe(candidate)
            if candidate:
                candidate[0] = raw[0]
                candidate[-1] = raw[-1]
            candidate = self.enforce_pure_pursuit_leadin(candidate)
            if candidate is None:
                continue
            # Finalize at exactly 0.10 m arc-length spacing for the downstream
            # controller's index-based lookahead calculation, then validate the
            # exact geometry that will actually be published.
            candidate = self._resample_polyline(candidate, self.output_spacing)
            if self.candidate_is_valid(candidate, tree, prediction):
                return candidate

        self.last_reject = self.last_reject or "no curvature-limited smooth spline"
        return None

    # ------------------------------------------------------------- goal / planning
    def dynamic_side_guides(self, side: str, horizon: float) -> List[List[float]]:
        guides: List[List[float]] = []
        # Bias samples through a gradual, vehicle-relative entry ramp.  They are
        # not mandatory points: RRT* still rejects them if current lidar says a
        # wall occupies the ramp.
        start = np.array((self.opp_x, self.opp_y), dtype=float)
        opp_forward = self._unit_from_yaw(self.opp_yaw)
        opp_lateral = np.array((-opp_forward[1], opp_forward[0])) if side == "left" else np.array((opp_forward[1], -opp_forward[0]))
        
        # Scale ramp dynamically so we don't plan straight through ego when close
        dist_to_ego = math.hypot(self.ego_x - self.opp_x, self.ego_y - self.opp_y)
        safe_ramp_limit = max(0.9, dist_to_ego - 0.4)
        ramp_distance = min(max(2.25, 2.50 * self.lateral_clearance), safe_ramp_limit)
        
        for fraction in np.linspace(0.20, 1.0, 5):
            smoothstep = 10.0 * fraction**3 - 15.0 * fraction**4 + 6.0 * fraction**5
            guides.append((start + fraction * ramp_distance * opp_forward +
                           smoothstep * 0.80 * self.lateral_clearance * opp_lateral).tolist())

        target_poses = max(3, int(math.ceil(horizon / self.sweep_time_step)))
        for t in np.linspace(0.0, horizon, target_poses):
            x, y, yaw = self.ego_pose_at(float(t))
            forward = self._unit_from_yaw(yaw)
            lateral = np.array((-forward[1], forward[0])) if side == "left" else np.array((forward[1], -forward[0]))
            guides.append((np.array((x, y)) + self.lateral_clearance * lateral).tolist())
        return guides

    def finalise_goal(self, prediction: Prediction, side: str, tree: KDTree) -> Optional[np.ndarray]:
        """Find a clear goal without ever shortening it back beside ego."""
        nominal = np.asarray(prediction.goal, dtype=float)
        forward = self._unit_from_yaw(prediction.ego_yaw_pred)
        lateral = np.array((-forward[1], forward[0])) if side == "left" else np.array((forward[1], -forward[0]))
        opp = np.array((self.opp_x, self.opp_y), dtype=float)
        # Search only outward sideways and forward.  In particular, do not use a
        # radial pull-back that can turn an overtake goal into a side-by-side goal.
        for forward_shift in (0.0, 0.12, 0.24):
            for lateral_shift in np.arange(0.0, 1.01, 0.05):
                candidate = nominal + forward_shift * forward + lateral_shift * lateral
                if float(np.linalg.norm(candidate - opp)) > self.max_goal_reach:
                    continue
                if not self.endpoint_overtakes_ego(candidate, prediction, strict=True):
                    continue
                clearance, _ = tree.query(candidate)
                if float(clearance) > self.rrt_clearance + self.goal_clearance_margin:
                    return candidate
        self.last_reject = "no collision-free endpoint ahead of predicted ego"
        return None

    def _plan_one_side(self, side: str) -> Tuple[Optional[List[List[float]]], Optional[Prediction]]:
        prediction = self.compute_predicted_goal(side)
        horizon = self.sweep_horizon(prediction.t_pred)
        tree = self.barrier_tree(horizon)
        if tree is None:
            self.last_reject = "no fresh lidar data"
            return None, None
        goal = self.finalise_goal(prediction, side, tree)
        if goal is None:
            return None, None
        prediction = Prediction(goal=goal, ego_pred=prediction.ego_pred,
                                t_pred=prediction.t_pred, ego_yaw_pred=prediction.ego_yaw_pred)

        # -------------------------------------------------------------------
        # 1. PRIMARY ATTEMPT: Aggressive Evasion Bézier
        # Generating using the exact 0.2 / 0.8 constraints requested
        # -------------------------------------------------------------------
        opp_pos = np.array((self.opp_x, self.opp_y), dtype=float)
        dist_to_ego = math.hypot(self.ego_x - self.opp_x, self.ego_y - self.opp_y)
        opp_forward = self._unit_from_yaw(self.opp_yaw)
        target_forward = self._unit_from_yaw(prediction.ego_yaw_pred)

        dist_to_goal = float(np.linalg.norm(prediction.goal - opp_pos))

        P0 = opp_pos
        # P1: Only push forward 20%. Forces immediate steering into passing lane!
        P1 = P0 + (dist_to_ego * 0.20) * opp_forward
        
        P3 = prediction.goal
        # P2: Pull backward 80%. Drags the curve into passing lane BEFORE reaching target!
        P2 = P3 - (self.overtake_distance * 0.80) * target_forward

        bezier_points = []
        resolution = max(20, int(dist_to_goal / (self.output_spacing / 2.0)))
        t_values = np.linspace(0.0, 1.0, resolution)
        for t in t_values:
            omt = 1.0 - t
            point = ((omt ** 3) * P0 + 
                     3 * (omt ** 2) * t * P1 + 
                     3 * omt * (t ** 2) * P2 + 
                     (t ** 3) * P3)
            bezier_points.append(point.tolist())

        bezier_candidate = self.dedupe(bezier_points)
        bezier_resampled = self._resample_polyline(bezier_candidate, self.output_spacing)
        
        # Enforce the pure pursuit lead-in so the curve starts perfectly straight,
        # guaranteeing it passes the strict initial heading check in candidate_is_valid.
        if len(bezier_resampled) > 1:
            bezier_resampled[1] = (np.array(bezier_resampled[0]) + self.output_spacing * opp_forward).tolist()

        if self.candidate_is_valid(bezier_resampled, tree, prediction):
            self._log("info", "Bézier curve used")
            return bezier_resampled, prediction

        # -------------------------------------------------------------------
        # 2. FALLBACK ATTEMPT: RRT* with smoothing
        # If the smooth Bézier is blocked by a wall, snake around it via RRT*
        # -------------------------------------------------------------------
        start = np.array((self.opp_x, self.opp_y), dtype=float)
        margin = max(1.5, 1.8 * self.lateral_clearance)
        guides = self.dynamic_side_guides(side, horizon)
        guide_array = np.asarray(guides + [start.tolist(), goal.tolist()], dtype=float)
        bounds = [
            min(float(start[0]), float(goal[0]), float(np.min(guide_array[:, 0]))) - margin,
            max(float(start[0]), float(goal[0]), float(np.max(guide_array[:, 0]))) + margin,
            min(float(start[1]), float(goal[1]), float(np.min(guide_array[:, 1]))) - margin,
            max(float(start[1]), float(goal[1]), float(np.max(guide_array[:, 1]))) + margin,
        ]
        planner = RRTStarPlanner(
            tree,
            bounds,
            expand_distance=0.45,
            collision_resolution=0.055,
            clearance=self.rrt_clearance,
            max_iterations=550,
            time_budget_s=0.045,
            guide_points=guides,
            guide_probability=0.38,
        )
        rrt_path = planner.plan(start, goal)
        if rrt_path is None or len(rrt_path) < 2:
            self.last_reject = "RRT* could not connect a safe path to the passing goal"
            return None, None

        guided = self._add_heading_guides(rrt_path, goal, prediction, side, planner)
        dense = self._resample_polyline(guided, 0.09)
        if not self.path_is_clear(dense, tree):
            self.last_reject = "RRT* polyline failed dense collision validation"
            return None, None
        smooth = self.smooth_validated_path(dense, tree, prediction)
        if smooth is None:
            self.last_reject = self.last_reject or "no smooth safe spline candidate"
            return None, None
        
        self._log("info", "RRT* fallback spline used")
        return smooth, prediction

    def generate_safe_spline(self) -> Tuple[Optional[List[List[float]]], Optional[Prediction], Optional[str]]:
        first = self.current_side if self.current_side in {"left", "right"} else "left"
        sides = (first, "right" if first == "left" else "left")
        for side in sides:
            path, prediction = self._plan_one_side(side)
            if path is not None:
                self.last_reject = ""
                return path, prediction, side
        return None, None, None

    # ---------------------------------------------------- active safety / replan
    def spline_intersects_current_ego(self, path: Sequence[Sequence[float]]) -> bool:
        """Literal intersection between the remaining spline and ego *now*.

        No future ego corridor, velocity extrapolation, or safety-envelope
        inflation is used here.  The question is intentionally only whether the
        currently published curve enters ego's current rectangular body.
        """
        samples = self._sample_path(path, self.active_intersection_resolution)
        if len(samples) == 0:
            return False
        half_length = self.car_length / 2.0
        half_width = self.car_width / 2.0
        for point in samples:
            if self.point_to_rect_distance(float(point[0]), float(point[1]),
                                           self.ego_x, self.ego_y, self.ego_yaw,
                                           half_length, half_width) <= 0.0:
                return True
        return False

    def active_path_is_safe(self) -> Tuple[bool, str]:
        """Cancel only for a current, observed geometric intersection.

        Future ego poses, virtual lidar occlusion bands, unknown-space rings,
        endpoint drift, tail probes, and stale scan age are deliberately *not*
        cancellation conditions for an already committed spline.
        """
        if self.active_path is None:
            return True, ""
        remaining = self.remaining_active_path()
        if remaining is None:
            # Completion is evaluated before this method in `_update_once`.
            # Do not turn an unexpected projection edge case into a false abort.
            return True, ""

        if self.spline_intersects_current_ego(remaining):
            return False, "current ego body intersects active spline"

        observed = self.current_observed_obstacle_points()
        if not observed:
            # Missing/stale lidar is not an obstacle intersection.  It prevents
            # generation of a new spline elsewhere, but does not flicker off the
            # existing committed one.
            return True, ""

        tree = KDTree(np.asarray(observed, dtype=float))
        samples = self._sample_path(remaining, self.active_intersection_resolution)
        distances, _ = tree.query(samples)
        minimum = float(np.min(distances)) if len(distances) else float("inf")
        if minimum <= self.active_intersection_tolerance:
            return False, f"current lidar obstacle intersects active spline ({minimum:.2f} m)"
        return True, ""

    def needs_replan(self) -> bool:
        """An active spline stays locked until collision cancellation/completion.

        The future ego prediction is still used when creating a new endpoint,
        but it is never used to continuously replace an already accepted path.
        """
        return self.active_path is None

    def _point_at_station(self, path: Sequence[Sequence[float]], station: float) -> np.ndarray:
        points = np.asarray(path, dtype=float)
        if len(points) == 1:
            return points[0]
        segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment)))
        station = float(np.clip(station, 0.0, cumulative[-1]))
        i = min(int(np.searchsorted(cumulative, station, side="right") - 1), len(segment) - 1)
        if segment[i] < 1e-9:
            return points[i]
        fraction = (station - cumulative[i]) / segment[i]
        return points[i] + fraction * (points[i + 1] - points[i])

    def new_path_is_continuous_enough(self, candidate: Sequence[Sequence[float]]) -> bool:
        """Avoid a large controller jump, but never publish a truncated path tail."""
        remaining = self.remaining_active_path()
        if remaining is None or len(remaining) < 2:
            return True
        old_length = self.path_length(remaining)
        new_length = self.path_length(candidate)
        overlap = min(1.4, old_length, new_length)
        if overlap < 0.35:
            return True
        distances = []
        for station in np.linspace(0.0, overlap, 5):
            distances.append(float(np.linalg.norm(
                self._point_at_station(remaining, float(station)) - self._point_at_station(candidate, float(station))
            )))
        return max(distances) <= self.max_switch_shift

    # --------------------------------------------------- immediate cancellation
    def mark_pass_complete(self) -> None:
        if not self.pass_complete:
            self._log("info", "Pass marked complete; overtake planning is disarmed until ego is clearly ahead again")
        self.pass_complete = True
        self.rearm_allowed_time = self.now_s() + self.rearm_cooldown

    def clear_spline(self, reason: str, *, completed: bool = False) -> None:
        """Delete all path state and publish an empty Path in this same timer tick."""
        had_path = self.active_path is not None
        self.active_path = None
        self.active_goal = None
        self.active_prediction = None
        self.active_created_at = 0.0
        if completed:
            self.mark_pass_complete()
        if had_path:
            self._log("info", f"Spline deleted immediately: {reason}")
        self.publish_empty_path(delete_marker=True)
        self.set_mode("FTG")

    # -------------------------------------------------------------- main callback
    def continuous_spline_update(self) -> None:
        try:
            self._update_once()
        except Exception as exc:  # Keep the ROS node alive; safety defaults to FTG.
            self._log("error", f"Planner tick exception: {exc}")
            self.clear_spline("planner exception", completed=False)

    def _update_once(self) -> None:
        self.tick_count += 1
        if not (self.ego_data_received and self.opp_data_received):
            return

        # Completion is odometry based and must be handled even if a scan is
        # late.  This remains the first active-path decision in every callback.
        completion = self.active_completion_reason()
        if completion is not None:
            self.clear_spline(completion, completed=True)
            return

        # Hard global gate: never generate or retain an overtake spline after
        # the commanded opponent has passed ego.
        if self.opponent_is_ahead():
            if self.active_path is not None:
                self.clear_spline("opponent is already ahead of ego", completed=True)
            else:
                self.mark_pass_complete()
                self.publish_empty_path(delete_marker=True)
                self.set_mode("FTG")
            return

        # A completed maneuver stays in FTG until the target is *firmly* ahead
        # again and the cooldown has elapsed.  This prevents an old path from
        # being regenerated while the cars are side-by-side after a pass.
        if self.pass_complete:
            if (self.longitudinal_to_ego() <= -self.pass_rearm_margin and
                    self.now_s() >= self.rearm_allowed_time):
                self.pass_complete = False
                self._log("info", "Ego is firmly ahead again; overtake planner re-armed")
            else:
                self.publish_empty_path(delete_marker=True)
                self.set_mode("FTG")
                return

        # No fresh scan means no new RRT* plan, but it is not evidence that the
        # committed spline intersects anything.  Hold the existing path instead
        # of cancelling/recreating it on a scan-timing hiccup.
        if not self.scan_is_fresh():
            if self.active_path is not None:
                self.refresh_active_path_output()
                self.set_mode("OVERTAKE_ACTIVE")
            else:
                self.publish_empty_path(delete_marker=True)
                self.set_mode("FTG")
            return

        now = self.now_s()
        distance = float(math.hypot(self.ego_x - self.opp_x, self.ego_y - self.opp_y))

        # A committed spline is never cancelled merely because a predicted goal,
        # distance threshold, or future ego pose changed.  It remains active
        # until completion/pass status or a literal *current* intersection says
        # otherwise.
        active_was_safe = False
        force_replan_now = False
        if self.active_path is not None:
            active_was_safe, unsafe_reason = self.active_path_is_safe()
            if not active_was_safe:
                # Actual current collision has been detected this callback.
                # Delete first, then allow one immediate replacement attempt.
                self.clear_spline(unsafe_reason, completed=False)
                force_replan_now = True
            else:
                self.refresh_active_path_output()
                self.set_mode("OVERTAKE_ACTIVE")
                return

        # These tighter conditions apply only to starting a new maneuver.  An
        # already-safe active path above is allowed to finish through the small
        # hysteresis band, but we never start one when opp is side-by-side/ahead.
        can_start_new_pass = (
            self.ego_is_ahead_enough_to_plan()
            and self.min_engage_distance <= distance <= self.max_engage_distance
        )
        if not can_start_new_pass:
            if active_was_safe and self.active_path is not None:
                self.refresh_active_path_output()
                self.set_mode("OVERTAKE_ACTIVE")
            else:
                self.publish_empty_path(delete_marker=True)
                self.set_mode("FTG")
            return

        # A failed plan with no committed safe path stays cleanly in FTG for a
        # short interval rather than alternately publishing a one-frame spline
        # and an empty path at 20 Hz.  A just-detected collision bypasses this
        # delay once so the required immediate replacement can be attempted.
        if self.active_path is None and not force_replan_now and now < self.next_plan_attempt_at:
            self.publish_empty_path(delete_marker=True)
            self.set_mode("FTG")
            return

        new_path, prediction, selected_side = self.generate_safe_spline()
        if new_path is not None and prediction is not None and selected_side is not None:
            # If a safe prior path exists, delay a large lateral jump rather than
            # publishing a chopped prefix (the old prefix was a cause of bad ends).
            if active_was_safe and self.active_path is not None and not self.new_path_is_continuous_enough(new_path):
                self._throttled("warning", "Safe replan differs too much; retaining the fully validated active spline")
                self.refresh_active_path_output()
                self.set_mode("OVERTAKE_ACTIVE")
                return

            self.active_path = self.dedupe(new_path)
            self.active_goal = np.asarray(prediction.goal, dtype=float)
            self.active_prediction = prediction
            self.active_created_at = now
            self.last_plan_time = now
            self.next_plan_attempt_at = 0.0
            self.current_side = selected_side
            self.publish_path(self.active_path)
            self.publish_prediction_marker(prediction)
            self.set_mode("OVERTAKE_ACTIVE")
            return

        # A failed replan may keep a previously validated path only if it was
        # still safe this exact tick.  An unsafe/cancelled path is never reused.
        if active_was_safe and self.active_path is not None:
            self._throttled("warning", "Replan failed; retaining only the still-safe active spline")
            self.refresh_active_path_output()
            self.set_mode("OVERTAKE_ACTIVE")
        else:
            self.next_plan_attempt_at = now + self.failed_plan_retry_period
            self._throttled("warning", f"No safe spline: {self.last_reject}")
            self.publish_empty_path(delete_marker=True)
            self.set_mode("FTG")

    # ---------------------------------------------------------------- publishing
    def refresh_active_path_output(self) -> None:
        """Refresh RViz without resetting the Pure Pursuit path every 50 ms."""
        if self.active_path is None:
            return
        now = self.now_s()
        if (self.path_heartbeat_period > 0.0 and
                now - self.last_path_publish_at >= self.path_heartbeat_period):
            self.publish_path(self.active_path)
            self.publish_prediction_marker(self.active_prediction)
            return
        if now - self.last_marker_publish_at >= self.marker_refresh_period:
            self.publish_spline_marker(self.active_path)
            self.publish_prediction_marker(self.active_prediction)

    def publish_path(self, path_points: Sequence[Sequence[float]]) -> None:
        points = self.dedupe(path_points)
        if len(points) < 2:
            self.publish_empty_path(delete_marker=True)
            return
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for i, point in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.05
            if i + 1 < len(points):
                delta = np.asarray(points[i + 1]) - np.asarray(point)
            else:
                delta = np.asarray(point) - np.asarray(points[i - 1])
            yaw = math.atan2(float(delta[1]), float(delta[0]))
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        self.overtake_path_pub.publish(message)
        self.last_path_publish_at = self.now_s()
        self.publish_spline_marker(points)

    def publish_empty_path(self, *, delete_marker: bool) -> None:
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        self.overtake_path_pub.publish(message)
        if delete_marker:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = message.header.stamp
            marker.ns = "overtake_spline"
            marker.id = 0
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)

    def publish_spline_marker(self, path_points: Sequence[Sequence[float]]) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "overtake_spline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.25, 0.95
        marker.lifetime = Duration(seconds=0.60).to_msg()
        for x, y in path_points:
            point = Point()
            point.x, point.y, point.z = float(x), float(y), 0.08
            marker.points.append(point)
        self.marker_pub.publish(marker)
        self.last_marker_publish_at = self.now_s()

    def publish_prediction_marker(self, prediction: Optional[Prediction]) -> None:
        if prediction is None:
            return
        stamp = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "overtake_goal"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(prediction.goal[0])
        marker.pose.position.y = float(prediction.goal[1])
        marker.pose.position.z = 0.16
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 1.0, 0.95
        marker.lifetime = Duration(seconds=0.35).to_msg()
        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OvertakeInterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()