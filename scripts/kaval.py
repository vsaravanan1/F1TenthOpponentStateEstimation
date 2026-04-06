#!/usr/bin/env python3
"""
subscribes to:
  /imm_path     - for the path prediction      
  /ego_odom     - for the oppornent's ground truth 
  /global_raceline    - for the global raceline spline 

It publishes:
  /defense_path      basically defense spline waypoints
  /defense_state     current KAVAL state - using for debugging 

State Machine
(trigs in Table 1)

States  (matching ARGOS):
  DISARM   – KAVAL and its internal states are continuously reset.
             Ego follows global raceline. 
 
  INIT     – KAVAL is armed. The local planner generates a position
             defense trajectory monitored for feasibility and safety.
             KAVAL remains here if the opponent does not attempt to pass
             OR if paht is not feasible.
 
  BLOCK    – KAVAL informs the ego to follow the defense path
 
  FALLBACK – Position defense unsuccessful. KAVAL slows down and merges
             onto the global raceline behind the opp.
 
  EXIT     – Transient state. Informs ARGOS the position defense is
             complete and resets KAVAL and the local planner.

Transitions:

  DISARM   ─(dist < trig0)-> INIT 
            if the car is withing the ego vehile range, then we need to init kaval 

  INIT     ──(dist > trig0)->  DISARM

            can disarm kaval if vehciles moves away 


  INIT     ──(dist < trig3, blocks < MAX_R2)──► BLOCK
            

  INIT     ──(defense not feasible)-> DISARM

  BLOCK    ──(ego ahead, dist > trig4)->  EXIT

            distance must reach a safe diatnace in front of other car to meerge back onto raceline 

  BLOCK    ──(opp ahead, dist > trig5)->  FALLBACK
            basicall if doesn not sucessuflly finish defnse, should not merge back onto the global raceline. 

  BLOCK    ──(blocks >= MAX_R2)->  FALLBACK


  FALLBACK ──(on_raceline)->  EXIT

            back to Exit when the dsitance is within min-max follow dist for tracking 
  EXIT     ──(reset complete)-> DISARM


--------------------------------------------------------------------------
Spline Construction (Equation 10 in ARGOS)

The defense interceptor spline I = {i0, i1, i2}

i0 = closest point on global raceline to ego  (start)
i1 = superprojected interception point         
i2 = closest point on global raceline ahead of opp   (merge target)

The superprojection of the opponent is:
  proj  = opp_pos + opp_vel * T_intercept
  super = proj  + k * lenght of car  * opp_heading_unit_vec

A quintic spline [i0, i1, i2]

"""

import math
import numpy as np
from enum import Enum, auto
 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
 
 

# Helper geometry funtions 
 
def closest_point_on_path(path_xy: np.ndarray, query: np.ndarray) -> np.ndarray:
    # return the point on the path that closest to the point that was passed in 
    dists = np.linalg.norm(path_xy - query, axis=1)
    return path_xy[np.argmin(dists)]
 
 
def point_at_distance(origin: np.ndarray, heading_rad: float, dist: float) -> np.ndarray:
    """k(o1, θ, d): project dist meters from origin along heading."""
    return origin + dist * np.array([math.cos(heading_rad), math.sin(heading_rad)])
 
 
def lateral_offset(origin: np.ndarray, toward: np.ndarray, dist: float) -> np.ndarray:
    """h(o1, o2, d): step dist meters from origin toward toward."""
    vec = toward - origin
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        return origin.copy()
    return origin + dist * vec / norm
 
 
def closer_bound(query: np.ndarray, left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """j(o1, o2, o3): return whichever of left/right is closer to query."""
    if np.linalg.norm(query - left) > np.linalg.norm(query - right):
        return left
    return right
 
 
def path_msg_to_xy(path: Path) -> np.ndarray:
    """Convert nav_msgs/Path to (N,2) numpy array."""
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return np.array(pts) if pts else np.zeros((0, 2))
 
 
def odom_to_state(odom: Odometry):
    """Extract (pos_xy, heading_rad, speed) from Odometry message."""
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    heading = math.atan2(siny_cosp, cosy_cosp)
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y
    speed = math.hypot(vx, vy)
    return np.array([x, y]), heading, speed
 
# ---------------------------------------------------

 
 
def quintic_coeffs(xs, vs, acc_s, xe, ve, acc_e, T):
    """
    Compute quintic polynomial coefficients for one axis.
 
    s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 
    Boundary conditions (Eq. 2):
      s(0) = xs,  ds/dt(0) = vs,  d²s/dt²(0) = acc_s
      s(T) = xe,  ds/dt(T) = ve,  d²s/dt²(T) = acc_e
    """
    a0 = xs
    a1 = vs
    a2 = acc_s / 2.0
 
    rhs = np.array([
        xe - a0 - a1 * T - a2 * T**2,
        ve - a1 - 2.0 * a2 * T,
        acc_e - 2.0 * a2,
    ])
    A = np.array([
        [T**3,       T**4,        T**5       ],
        [3.0 * T**2, 4.0 * T**3,  5.0 * T**4 ],
        [6.0 * T,   12.0 * T**2, 20.0 * T**3 ],
    ])
    try:
        a345 = np.linalg.solve(A, rhs)
    except np.linalg.LinAlgError:
        a345 = np.zeros(3)
    return np.array([a0, a1, a2, *a345])
 
 
def eval_quintic(coeffs, t):
    """Evaluate quintic polynomial at scalar t."""
    return sum(c * t**i for i, c in enumerate(coeffs))
 
 
def quintic_spline_waypoints(
    start_pos, start_vel_xy, start_acc_xy,
    end_pos, end_vel_xy, end_acc_xy,
    T, n_points=30,
):
    """
    Build a list of (x,y) waypoints along the quintic spline
    from start_pos to end_pos with the given boundary conditions.
    """
    cx = quintic_coeffs(
        start_pos[0], start_vel_xy[0], start_acc_xy[0],
        end_pos[0],   end_vel_xy[0],   end_acc_xy[0], T,
    )
    cy = quintic_coeffs(
        start_pos[1], start_vel_xy[1], start_acc_xy[1],
        end_pos[1],   end_vel_xy[1],   end_acc_xy[1], T,
    )
    ts = np.linspace(0.0, T, n_points)
    return [(eval_quintic(cx, t), eval_quintic(cy, t)) for t in ts]
 
 
"""
    state Machine stuff from here 
"""
class KAVALState(Enum):
    DISARM   = auto()   # ego follows global raceline
    INIT     = auto()   # Armed; defense trajectory computed and monitored
    BLOCK    = auto()   # following the defesne traj
    FALLBACK = auto()   # Defense failed; slow down, merge behind opponent
    EXIT     = auto()   # Transient; defense complete, reset KAVAL
 
 
 # actual state machine 
class KAVALDefensePlanner:
    """
    KAVAL Defender automaton (Fig. 11, ARGOS paper).

    TRIG0   opponent tracking radius-20.0 m)
    TRIG1   minimum follow distance - 1.5 m)
    TRIG2   maximum follow distance- 4.0 m)
    TRIG3   block start threshold-3.0 m)
    TRIG4   maneuver complete — ego ahead-1.0 m)
    TRIG5   maneuver failed   — opp ahead-2.0 m)
 
    MAX_R2  = max block attempts allowed (Rule R2)
    LWB     = car length in meters (F1Tenth 1:10 scale ≈ 0.5 m)
    K       = superprojection multiplier (k * LWB ahead of projected opp)
    """
 
    # CHANGE THESE CONSTANTS LATER, MUST TUNE THEM 
    TRIG0   = 20.0
    TRIG1   =  1.5
    TRIG2   =  4.0
    TRIG3   =  3.0
    TRIG4   =  1.0
    TRIG5   =  2.0
    LWB     =  0.5
    K       =  2.0
    MAX_R2  =  2
 
    def __init__(self):
        self._reset()
 
    def _reset(self):
        """Reset all internal state — called on entry to DISARM and EXIT."""
        self.state = KAVALState.DISARM
        self.block_count = 0
        self.active_spline: list[tuple] = []
        self._exit_tick = 0      # counts ticks spent in EXIT before DISARM
 
    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
 
    def update(
        self,
        ego_pos: np.ndarray,
        ego_heading: float,
        ego_speed: float,
        opp_pos: np.ndarray,
        opp_heading: float,
        opp_speed: float,
        opp_predicted_path: np.ndarray,
        raceline: np.ndarray,
        track_left: np.ndarray,
        track_right: np.ndarray,
    ) -> tuple[list[tuple], str]:
        """
        Step the KAVAL automaton once.
 
        Returns
        -------
        waypoints : list of (x, y) — local reference path for MPC
        state_str : current KAVAL state name string
        """
        dist = float(np.linalg.norm(ego_pos - opp_pos))
 
        # ── DISARM ──────────────────────────────────────────────────────
        # Continuously reset internal state.
        # Transition to INIT the moment opponent enters tracking range.
        if self.state == KAVALState.DISARM:
            self.block_count = 0
            self.active_spline = []
            self._exit_tick = 0
            if dist < self.TRIG0:
                self.state = KAVALState.INIT
                # Pre-arm: compute initial defense spline
                self.active_spline = self._compute_defense_spline(
                    ego_pos, ego_heading, ego_speed,
                    opp_pos, opp_heading, opp_speed,
                    raceline, track_left, track_right,
                )
 
        # ── INIT ─────────────────────────────────────────────────────────
        # Armed. Continuously recompute and monitor defense spline.
        # Ego stays on global raceline — no deviation yet.
        elif self.state == KAVALState.INIT:
            # Recompute every tick so the spline tracks the opponent's
            # evolving motion vector (feasibility monitoring per paper)
            self.active_spline = self._compute_defense_spline(
                ego_pos, ego_heading, ego_speed,
                opp_pos, opp_heading, opp_speed,
                raceline, track_left, track_right,
            )
            feasible = self._is_spline_feasible(
                self.active_spline, track_left, track_right
            )
 
            if dist > self.TRIG0:
                # Opponent left tracking range
                self.state = KAVALState.DISARM
            elif not feasible or self.block_count >= self.MAX_R2:
                # Defense not feasible or block budget exhausted
                self.state = KAVALState.DISARM
            elif dist < self.TRIG3:
                # Opponent close enough — commit to block
                self.block_count += 1
                self.state = KAVALState.BLOCK
 
        # ── BLOCK ─────────────────────────────────────────────────────────
        # Ego tracks interception spline (deviates from global raceline).
        elif self.state == KAVALState.BLOCK:
            ego_ahead = self._is_ego_ahead_on_raceline(ego_pos, opp_pos, raceline)
 
            if ego_ahead and dist > self.TRIG4:
                # Success: ego ahead with safe gap → EXIT
                self.state = KAVALState.EXIT
            elif (not ego_ahead and dist > self.TRIG5) or \
                 self.block_count >= self.MAX_R2:
                # Failure: opponent passed, or budget exhausted → FALLBACK
                self.state = KAVALState.FALLBACK
 
        # ── FALLBACK ──────────────────────────────────────────────────────
        # Slow and merge onto raceline BEHIND opponent.
        # Ego dynamics affected — different spline than global raceline.
        # Transition to EXIT once ego is back on the raceline AND the
        # distance to the opponent is within the [TRIG1, TRIG2] follow
        # window — i.e. safely trailing at the correct gap.
        elif self.state == KAVALState.FALLBACK:
            merge_target = closest_point_on_path(raceline, ego_pos)
            on_raceline  = np.linalg.norm(ego_pos - merge_target) < 0.3
            in_follow_window = self.TRIG1 <= dist <= self.TRIG2
            if on_raceline and in_follow_window:
                self.state = KAVALState.EXIT
 
        # ── EXIT ──────────────────────────────────────────────────────────
        # Transient: hold for ~3 ticks (150 ms) so ARGOS can observe it,
        # then reset KAVAL and local planner back to DISARM.
        elif self.state == KAVALState.EXIT:
            self._exit_tick += 1
            if self._exit_tick >= 3:
                self._reset()
 
        # ── Output waypoints ──────────────────────────────────────────────
        # Only BLOCK and FALLBACK deviate ego from global raceline.
        if self.state == KAVALState.BLOCK:
            waypoints = self.active_spline
        elif self.state == KAVALState.FALLBACK:
            waypoints = self._fallback_spline(
                ego_pos, ego_heading, ego_speed, opp_pos, raceline
            )
        else:
            # DISARM, INIT, EXIT — global raceline, no deviation
            waypoints = self._raceline_lookahead(ego_pos, raceline, n=40)
 
        return waypoints, self.state.name
 
    # ------------------------------------------------------------------
    # Defense spline construction (Equation 10 from ARGOS)
    # ------------------------------------------------------------------
 
    def _compute_defense_spline(
        self,
        ego_pos, ego_heading, ego_speed,
        opp_pos, opp_heading, opp_speed,
        raceline, track_left, track_right,
    ) -> list[tuple]:
        """
        Build interceptor spline I = {i0, i1, i2} per Eq. 10.
 
          i0 = g(R_ego, W)
               Ego's closest point on raceline — spline start.
 
          i1 = h(k(R_opp, θ, v*Tp), g(k(R_opp, v*Tp), j(R_opp, BL, BR)))
               Superprojected opponent pose shifted toward nearest bound —
               the interception / block target (Fig. 5 geometry).
 
          i2 = g(k(R_opp, θ, v*Tp + LCD), W)
               Raceline merge point past the projected opponent.
        """
        i0 = closest_point_on_path(raceline, ego_pos)
 
        # Intercept time from relative closing speed
        rel_speed = max(ego_speed - opp_speed, 0.5)
        dist = float(np.linalg.norm(ego_pos - opp_pos))
        T_p = dist / rel_speed
 
        opp_heading_unit = np.array([math.cos(opp_heading), math.sin(opp_heading)])
 
        # Project opponent forward T_p seconds
        opp_proj = opp_pos + opp_speed * T_p * opp_heading_unit
 
        # Superprojection: K*LWB further ahead along opponent heading
        opp_super = opp_proj + self.K * self.LWB * opp_heading_unit
 
        # Nearest track boundary to superprojected point
        nearest_left  = track_left[np.argmin(np.linalg.norm(track_left  - opp_super, axis=1))]
        nearest_right = track_right[np.argmin(np.linalg.norm(track_right - opp_super, axis=1))]
        nearest_bound = closer_bound(opp_super, nearest_left, nearest_right)
 
        # i1: shift superprojection toward wall by one car width
        i1 = lateral_offset(opp_super, nearest_bound, self.LWB)
 
        # i2: raceline merge point one extra car-length past the block
        opp_proj_far = opp_proj + (self.K + 1.0) * self.LWB * opp_heading_unit
        i2 = closest_point_on_path(raceline, opp_proj_far)
 
        # Fit quintic spline through {i0, i1, i2}
        seg_len = np.linalg.norm(i1 - i0) + np.linalg.norm(i2 - i1)
        T_total = max(seg_len / max(ego_speed, 0.5), 0.5)
 
        ego_vel_xy = ego_speed * np.array([math.cos(ego_heading), math.sin(ego_heading)])
        end_dir    = i2 - i1
        end_vel_xy = ego_speed * end_dir / (np.linalg.norm(end_dir) + 1e-6)
 
        return quintic_spline_waypoints(
            start_pos=i0,  start_vel_xy=ego_vel_xy, start_acc_xy=np.zeros(2),
            end_pos=i2,    end_vel_xy=end_vel_xy,   end_acc_xy=np.zeros(2),
            T=T_total, n_points=60,
        )
 
    # ------------------------------------------------------------------
    # Feasibility check (gates INIT → BLOCK transition)
    # ------------------------------------------------------------------
 
    def _is_spline_feasible(
        self,
        spline: list[tuple],
        track_left: np.ndarray,
        track_right: np.ndarray,
        safety_margin: float = 0.15,
    ) -> bool:
        """
        True if every spline waypoint is inside track bounds
        with at least safety_margin clearance.
        """
        if not spline:
            return False
        for x, y in spline:
            pt = np.array([x, y])
            d_left  = float(np.min(np.linalg.norm(track_left  - pt, axis=1)))
            d_right = float(np.min(np.linalg.norm(track_right - pt, axis=1)))
            if d_left < safety_margin or d_right < safety_margin:
                return False
        return True
 
    # ------------------------------------------------------------------
    # FALLBACK spline — merge behind opponent onto raceline
    # ------------------------------------------------------------------
 
    def _fallback_spline(
        self,
        ego_pos, ego_heading, ego_speed,
        opp_pos, raceline, n=40,
    ) -> list[tuple]:
        """
        FALLBACK: quintic blend to a point TRIG2 meters behind the
        opponent on the raceline (safe trailing gap after failed block).
        """
        opp_rl_idx = int(np.argmin(np.linalg.norm(raceline - opp_pos, axis=1)))
        acc, target_idx = 0.0, max(opp_rl_idx - 1, 0)
        for i in range(opp_rl_idx, 0, -1):
            acc += np.linalg.norm(raceline[i] - raceline[i - 1])
            if acc >= self.TRIG2:
                target_idx = i
                break
 
        merge_target = raceline[target_idx]
        T = max(np.linalg.norm(merge_target - ego_pos) / max(ego_speed, 0.3), 0.3)
 
        ego_vel_xy = ego_speed * np.array([math.cos(ego_heading), math.sin(ego_heading)])
        end_dir    = merge_target - ego_pos
        end_vel_xy = ego_speed * end_dir / (np.linalg.norm(end_dir) + 1e-6)
 
        return quintic_spline_waypoints(
            start_pos=ego_pos,    start_vel_xy=ego_vel_xy, start_acc_xy=np.zeros(2),
            end_pos=merge_target, end_vel_xy=end_vel_xy,   end_acc_xy=np.zeros(2),
            T=T, n_points=n,
        )
 
    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------
 
    def _raceline_lookahead(self, ego_pos, raceline, n=40) -> list[tuple]:
        """Return the next n raceline waypoints ahead of ego."""
        idx = int(np.argmin(np.linalg.norm(raceline - ego_pos, axis=1)))
        seg = raceline[idx: idx + n]
        return [(float(p[0]), float(p[1])) for p in seg]
 
    def _is_ego_ahead_on_raceline(self, ego_pos, opp_pos, raceline) -> bool:
        """True if ego is further along the raceline than the opponent."""
        ego_idx = int(np.argmin(np.linalg.norm(raceline - ego_pos, axis=1)))
        opp_idx = int(np.argmin(np.linalg.norm(raceline - opp_pos, axis=1)))
        return ego_idx > opp_idx
 
 
# ROS 2 Node
 
class DefensiveManeuverNode(Node):
    """
    Publishes /defense_path at 20 Hz for the downstream MPC tracker.
    """
 
    def __init__(self):
        super().__init__('defensive_maneuver_node')
 
        self.sub_imm   = self.create_subscription(Path, '/imm_path',self._cb_imm, 10)
        self.sub_odom  = self.create_subscription( Odometry, '/ego_odom', self._cb_odom,10)
        self.sub_raceline = self.create_subscription(Path,'/global_raceline', self._cb_raceline, 10)
 
        self.pub_path  = self.create_publisher(Path,   '/defense_path',  10)
        self.pub_state = self.create_publisher(String, '/defense_state', 10)
 
        self.planner     = KAVALDefensePlanner() # createing an isntance of the SM in the Node 
        self.ego_odom    = None
        self.imm_path    = None
        self.raceline    = None
        self.track_left  = None
        self.track_right = None
 
        self.timer = self.create_timer(0.05, self._plan_and_publish)  # 20Hz
        self.get_logger().info("KAVAL started.........")
 
    def _cb_odom(self, msg: Odometry):
        self.ego_odom = msg
 
    def _cb_imm(self, msg: Path):
        self.imm_path = msg
 
    def _cb_raceline(self, msg: Path):
        self.raceline = msg
        rl = path_msg_to_xy(msg)
        if len(rl) > 1 and self.track_left is None:
            diffs  = np.diff(rl, axis=0)
            heads  = np.arctan2(diffs[:, 1], diffs[:, 0])
            heads  = np.append(heads, heads[-1])
            perp   = heads + math.pi / 2.0
            offset = 0.7   # NEED TO REPLACE LATER THIS IS IN METERS ____________________________________ ( jalf trakc width)
            self.track_left  = rl + offset * np.column_stack(
                [np.cos(perp), np.sin(perp)])
            self.track_right = rl - offset * np.column_stack(
                [np.cos(perp), np.sin(perp)])
 
    def _plan_and_publish(self):
        if self.ego_odom is None or self.raceline is None:
            return
 
        ego_pos, ego_heading, ego_speed = odom_to_state(self.ego_odom)
        raceline_xy = path_msg_to_xy(self.raceline)
 
        if self.imm_path and len(self.imm_path.poses) >= 2:
            opp_predicted = path_msg_to_xy(self.imm_path)
            opp_pos     = opp_predicted[0]
            d           = opp_predicted[1] - opp_predicted[0]
            opp_heading = math.atan2(d[1], d[0])
            opp_speed   = float(np.linalg.norm(d)) / 0.05   # dt = 50 ms
        else:
            self._publish_waypoints(
                self.planner._raceline_lookahead(ego_pos, raceline_xy),
                KAVALState.DISARM.name,
            )
            return
 
        track_left  = self.track_left  if self.track_left  is not None else raceline_xy
        track_right = self.track_right if self.track_right is not None else raceline_xy
 
        try:
            waypoints, state_name = self.planner.update(
                ego_pos=ego_pos,
                ego_heading=ego_heading,
                ego_speed=ego_speed,
                opp_pos=opp_pos,
                opp_heading=opp_heading,
                opp_speed=opp_speed,
                opp_predicted_path=opp_predicted,
                raceline=raceline_xy,
                track_left=track_left,
                track_right=track_right,
            )
        except Exception as e:
            self.get_logger().warn(f"KAVAL planner error: {e}")
            self._publish_waypoints(
                self.planner._raceline_lookahead(ego_pos, raceline_xy),
                KAVALState.DISARM.name,
            )
            return
 
        self._publish_waypoints(waypoints, state_name)
        self.get_logger().debug(
            f"KAVAL state: {state_name} | waypoints: {len(waypoints)}"
        ) # for my debugging kill me 
 
    def _publish_waypoints(self, waypoints: list[tuple], state_name: str):
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for x, y in waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            path_msg.poses.append(ps)
        self.pub_path.publish(path_msg)
 
        state_msg = String()
        state_msg.data = state_name
        self.pub_state.publish(state_msg)
 
 
# entry point 
def main(args=None):
    rclpy.init(args=args)
    node = DefensiveManeuverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
