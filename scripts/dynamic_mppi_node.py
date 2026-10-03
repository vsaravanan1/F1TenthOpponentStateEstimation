#!/usr/bin/python3
"""
dynamic_mppi_node.py

A SELF-CONTAINED copy of the MPPI controller that can be handed a divert spline at
runtime, follow it, then generate its own smooth return to the centerline.

It does NOT import from or modify the working f1tenth_mppi package.

Reference handling:
  * self.active_waypoints is what MPPI tracks. Default = centerline.
  * On a message on `divert_topic` (Float64MultiArray, "csv-like" [x,y] or [x,y,psi,v]
    in the map frame), the node:
       1) uses the points VERBATIM as the divert (no heavy geometry here),
       2) generates ONE return spline from the divert end back to the centerline
          (the only geometry MPPI runs itself),
       3) splices centerline -> divert -> return -> centerline into active_waypoints.
  * A tiny state machine (RACELINE -> DIVERT -> RETURN -> RACELINE) reverts to the
    centerline once the car passes the end of the return. The game-theory layer can
    later decide WHEN to publish a divert; this node just follows what it is given.

The MPPI math is unchanged from the original; only get_nearest_waypoint reads
active_waypoints instead of a fixed array.
"""

import math
import numpy as np
from scipy.ndimage import binary_dilation
from typing import Tuple

import rclpy
import tf2_ros
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# Reuse the project's real KBM (identical dynamics; read-only import, does not
# modify f1tenth_mppi). If you ever want zero shared imports, drop a copy of
# dynamics_models.py into f1tenth_mppi_dynamic and import it here instead.
from f1tenth_mppi.dynamics_models import KBM
from f1tenth_mppi_dynamic.reference_manager import ReferencePlanner
from f1tenth_mppi_dynamic.io_utils import (load_waypoints, unpack_waypoints, line_marker)

RACELINE, DIVERT, RETURN = "RACELINE", "DIVERT", "RETURN"


class DynamicMPPI(Node):
    def __init__(self):
        super().__init__("dynamic_mppi")
        self.info = self.get_logger()
        self._init_params()
        if self.get_parameter("drive_topic").value is None:
            self.info.error("No parameters set, use --ros-args --params-file <FILE>")
            raise SystemExit(1)

        P = lambda n: self.get_parameter(n).value

        self.model = KBM(P("wheelbase"), P("min_throttle"), P("max_throttle"),
                         P("max_steer"), P("dt"))

        # occupancy grid (vehicle frame), identical setup to the original
        self.og = OccupancyGrid()
        self.og.header.frame_id = P("vehicle_frame")
        self.og.info.resolution = P("cost_map_res")
        self.og.info.width = P("cost_map_width")
        self.og.info.height = P("cost_map_width")
        self.og.info.origin.position.x = 0.0
        self.og.info.origin.position.y = -(self.og.info.height * self.og.info.resolution) / 2

        self.u_prev = np.zeros((P("steps_trajectories"), 2))

        # --- load + normalise the centerline exactly like the original node ---
        wp = load_waypoints(P("waypoint_path"))
        mn, mx = np.min(wp[:, 3]), np.max(wp[:, 3])
        wp[:, 3] = (wp[:, 3] - mn) / (mx - mn)
        wp[:, 3] = wp[:, 3] * (P("max_throttle") - P("min_throttle")) + P("min_throttle")
        self.raceline = wp
        self.active_waypoints = self.raceline          # what MPPI tracks
        self.planner = ReferencePlanner(self.raceline, closed=True)

        # state machine
        self.state = RACELINE
        self.maneuver = None   # dict with divert_end_idx, return_end_idx, return_end_xy

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, P("drive_topic"), 10)
        self.occ_pub = self.create_publisher(OccupancyGrid, P("occupancy_topic"), 10)
        self.marker_pub = self.create_publisher(MarkerArray, P("marker_topic"), 10)
        self.state_pub = self.create_publisher(String, P("state_topic"), 10)

        # divert input ("csv-like" waypoints from converter or injector)
        self.divert_sub = self.create_subscription(
            Float64MultiArray, P("divert_topic"), self.divert_cb, 10)

        # synchronized scan + pose (unchanged)
        self.scan_sub = Subscriber(self, LaserScan, P("scan_topic"))
        self.pose_sub = Subscriber(self, Odometry, P("pose_topic"))
        self.sync = ApproximateTimeSynchronizer([self.scan_sub, self.pose_sub], 10, 0.1)
        self.sync.registerCallback(self.callback)

        if P("visualize"):
            self._publish_reference_markers()
        self.info.info("dynamic_mppi node initialized (state=RACELINE)")

    def _init_params(self):
        self.declare_parameters("", [
            ("visualize", None), ("waypoint_path", None), ("vehicle_frame", None),
            ("drive_topic", None), ("occupancy_topic", None), ("marker_topic", None),
            ("pose_topic", None), ("scan_topic", None),
            ("wheelbase", None), ("min_throttle", None), ("max_throttle", None),
            ("max_steer", None), ("dt", None),
            ("num_trajectories", None), ("steps_trajectories", None),
            ("v_sigma", None), ("omega_sigma", None), ("lambda", None),
            ("cost_map_width", None), ("cost_map_res", None), ("occupancy_dilation", None),
            # ---- dynamic-MPPI additions ----
            ("divert_topic", "/divert_waypoints"),
            ("state_topic", "/mppi_state"),
            ("divert_speed", 0.85),       # throttle used if a divert msg has no v column
            ("return_len", 4.0),          # nominal rejoin length (m)
            ("return_spacing", 0.1),      # resample spacing for divert/return (m)
            ("allow_interrupt", False),   # accept a new divert mid-maneuver?
            ("revert_margin_pts", 2),     # revert this many pts before the return end
        ])

    # ------------------------------------------------------------------ #
    # Divert intake  (runs once per message, NOT in the MPPI hot loop)
    # ------------------------------------------------------------------ #
    def divert_cb(self, msg: Float64MultiArray):
        if self.state != RACELINE and not self.get_parameter("allow_interrupt").value:
            self.info.warn("divert received but still maneuvering; ignoring "
                           "(set allow_interrupt:=true to override)")
            return

        arr = unpack_waypoints(msg)
        if arr.shape[0] < 2:
            self.info.warn("divert has <2 points; ignoring")
            return

        # Accept [x,y] or [x,y,psi,v]. If only xy, build clean waypoints (light:
        # resample + tangent heading; NOT the Frenet geometry we keep out of MPPI).
        spacing = self.get_parameter("return_spacing").value
        if arr.shape[1] >= 4:
            divert = arr[:, :4].copy()
        else:
            divert = ReferencePlanner.path_to_waypoints(
                arr[:, :2], self.get_parameter("divert_speed").value, spacing)

        # heading at the divert end, then generate the return (MPPI's own geometry)
        end_xy = divert[-1, :2]
        d_end = divert[-1, :2] - divert[-3, :2] if len(divert) >= 3 else divert[-1, :2] - divert[-2, :2]
        end_heading = math.atan2(d_end[1], d_end[0])
        start_v = float(divert[-1, 3])
        ret, rinfo = self.planner.build_return(
            end_xy, end_heading,
            return_len=self.get_parameter("return_len").value,
            spacing=spacing, start_v=start_v)

        if ret is None:
            self.info.warn(f"return infeasible ({rinfo.get('reason','?')}); "
                           "ignoring divert, staying on centerline")
            return

        try:
            active, info = self.planner.splice(divert, ret)
        except ValueError as e:
            self.info.warn(f"splice failed: {e}; ignoring divert")
            return

        self.active_waypoints = active
        self.maneuver = info
        self.state = DIVERT
        self.info.info(f"DIVERT accepted: {info['n_divert']} divert + {info['n_return']} "
                       f"return pts; rejoin at idx {info['return_end_idx']}")
        if self.get_parameter("visualize").value:
            self._publish_reference_markers(info)

    def _update_state(self, car_xy):
        """Advance the state machine from the car's progress along active_waypoints."""
        if self.state == RACELINE or self.maneuver is None:
            return
        # nearest index of the car on the active array (cheap: 1 x N)
        d2 = np.sum((self.active_waypoints[:, :2] - car_xy) ** 2, axis=1)
        idx = int(np.argmin(d2))
        margin = int(self.get_parameter("revert_margin_pts").value)

        if self.state == DIVERT and idx >= self.maneuver["divert_end_idx"]:
            self.state = RETURN
            self.info.info("RETURN: following self-generated rejoin")
        if idx >= self.maneuver["return_end_idx"] - margin:
            self.state = RACELINE
            self.active_waypoints = self.raceline
            self.maneuver = None
            self.info.info("RACELINE: merge complete, back on centerline")
            if self.get_parameter("visualize").value:
                self._publish_reference_markers()

    # ------------------------------------------------------------------ #
    # Main control callback  (MPPI math unchanged from the original)
    # ------------------------------------------------------------------ #
    def callback(self, scan_msg: LaserScan, pose_msg: Odometry):
        self.create_occupancy_grid(scan_msg)

        qx = pose_msg.pose.pose.orientation.x; qy = pose_msg.pose.pose.orientation.y
        qz = pose_msg.pose.pose.orientation.z; qw = pose_msg.pose.pose.orientation.w
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        if yaw < 0:
            yaw += 2 * np.pi
        x0 = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, yaw])

        # state machine update (follow -> return -> raceline)
        self._update_state(x0[:2])
        sm = String(); sm.data = self.state; self.state_pub.publish(sm)

        NT = self.get_parameter("num_trajectories").value
        ST = self.get_parameter("steps_trajectories").value
        u = self.u_prev

        mu = np.zeros(2)
        sigma = np.array([[self.get_parameter("v_sigma").value, 0.0],
                          [0.0, self.get_parameter("omega_sigma").value]])
        epsilon = np.random.multivariate_normal(mu, sigma, (NT - 1, ST))
        epsilon = np.vstack((np.zeros((1, ST, 2)), epsilon))

        S = np.zeros(NT)
        v = np.zeros((NT, ST, 2))
        x = np.zeros((NT, ST + 1, 3))
        x[:, 0] = x0

        for j in range(1, ST):
            v[:, j - 1] = u[j - 1] + epsilon[:, j - 1]
            v[:, j - 1, 0] = np.clip(v[:, j - 1, 0],
                                     self.get_parameter("min_throttle").value,
                                     self.get_parameter("max_throttle").value)
            v[:, j - 1, 1] = np.clip(v[:, j - 1, 1],
                                     -self.get_parameter("max_steer").value,
                                     self.get_parameter("max_steer").value)
            epsilon[:, j - 1] = v[:, j - 1] - u[j - 1]
            x[:, j] = self.model.predict_euler(x[:, j - 1], v[:, j - 1])
            S += self.compute_cost(x[:, j], v[:, j - 1, 0], pose_msg).squeeze()

        S += self.compute_cost(x[:, j], v[:, j - 1, 0], pose_msg).squeeze() * 10
        w = self.compute_weights(S)
        w_epsilon = np.sum(np.multiply(w[:, np.newaxis, np.newaxis], epsilon), axis=0)
        w_epsilon = self.moving_average(w_epsilon, 4)
        u += w_epsilon
        u[:, 0] = np.clip(u[:, 0], self.get_parameter("min_throttle").value,
                          self.get_parameter("max_throttle").value)
        u[:, 1] = np.clip(u[:, 1], -self.get_parameter("max_steer").value,
                          self.get_parameter("max_steer").value)

        self.u_prev[:-1] = u[1:]
        self.u_prev[-1] = u[-1]

        # optimal trajectory viz
        if self.get_parameter("visualize").value:
            traj = np.zeros((ST + 1, 3)); traj[0] = x0
            for i in range(ST):
                traj[i + 1] = self.model.predict_euler(np.expand_dims(traj[i], 0),
                                                       np.expand_dims(u[i], 0)).squeeze()
            self._publish_opt(traj)

        drive = AckermannDriveStamped()
        drive.drive.speed = float(u[0, 0])
        drive.drive.steering_angle = float(u[0, 1])
        self.drive_pub.publish(drive)

    # ------------------------------------------------------------------ #
    # Cost (unchanged) -- only get_nearest_waypoint reads active_waypoints
    # ------------------------------------------------------------------ #
    def compute_cost(self, x_t, v_t, pose_msg):
        weights = [13.5, 13.5, 12.0, 5.0]   # yaw weight raised now that it's correct
        xx, yy, ya = np.hsplit(x_t, 3)
        vv = np.expand_dims(v_t, 1)
        _, rx, ry, ryaw, rv = self.get_nearest_waypoint(xx, yy)

        # stored psi -> true heading is (pi/2 - psi)
        ryaw = np.pi / 2.0 - ryaw

        # properly wrapped heading error in (-pi, pi]; handles all wrap cases
        yaw_err = np.arctan2(np.sin(ya - ryaw), np.cos(ya - ryaw))

        cost = (weights[0] * (xx - rx) ** 2 + weights[1] * (yy - ry) ** 2 +
                weights[2] * yaw_err ** 2 + weights[3] * (vv - rv) ** 2)
        cost += np.expand_dims(self.is_collided(x_t, pose_msg), 1) * 1.0e10
        return cost

    def get_nearest_waypoint(self, x, y):
        cur = np.hstack((x, y))
        NT = self.get_parameter("num_trajectories").value
        wp = self.active_waypoints                              # <-- active reference
        cur_r = np.repeat(np.expand_dims(cur, 1), len(wp), axis=1)
        wp_r = np.repeat(np.expand_dims(wp[:, :2], 0), NT, axis=0)
        dist = np.linalg.norm(cur_r - wp_r, axis=2)
        idx = np.argmin(dist, axis=1)
        wx, wy, wyaw, wv = np.hsplit(wp[idx], 4)
        return idx, wx, wy, wyaw, wv

    def is_collided(self, x_t, pose_msg):
        occ = np.array(self.og.data).reshape((self.og.info.height, self.og.info.width))
        q = pose_msg.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        T = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        pos = np.dot(x_t[:, :2] - T, R)
        pix = pos / self.og.info.resolution
        pix[:, 1] = pix[:, 1] + (self.og.info.height / 2)
        pix = np.clip(pix, 0, self.og.info.width - 1)
        return occ[pix[:, 1].astype(int), pix[:, 0].astype(int)] > 0

    def compute_weights(self, S):
        rho = S.min()
        lam = self.get_parameter("lambda").value
        eta = np.sum(np.exp((-1.0 / lam) * (S - rho)))
        return (1.0 / eta) * np.exp((-1.0 / lam) * (S - rho))

    def moving_average(self, xx, window_size):
        b = np.ones(window_size) / window_size
        dim = xx.shape[1]
        out = np.zeros(xx.shape)
        for d in range(dim):
            out[:, d] = np.convolve(xx[:, d], b, mode="same")
            n_conv = math.ceil(window_size / 2)
            out[0, d] *= window_size / n_conv
            for i in range(1, n_conv):
                out[i, d] *= window_size / (i + n_conv)
                out[-i, d] *= window_size / (i + n_conv - (window_size % 2))
        return out

    def create_occupancy_grid(self, scan_msg):
        grid = np.zeros((self.og.info.width, self.og.info.width), dtype=int)
        angles = scan_msg.angle_min + scan_msg.angle_increment * np.arange(len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)
        xc = np.round((ranges * np.sin(angles)) / self.og.info.resolution + self.og.info.width / 2).astype(int)
        yc = np.round((ranges * np.cos(angles)) / self.og.info.resolution).astype(int)
        m = (xc > 0) & (xc < self.og.info.width) & (yc > 0) & (yc < self.og.info.height)
        grid[xc[m], yc[m]] = 100
        k = self.get_parameter("occupancy_dilation").value
        grid = binary_dilation(grid, structure=np.ones((k, k), dtype=bool)).astype(int) * 100
        self.og.data = grid.flatten().tolist()
        if self.get_parameter("visualize").value:
            self.og.header.stamp = self.get_clock().now().to_msg()
            self.occ_pub.publish(self.og)
        return grid

    # ------------------------------- viz -------------------------------- #
    def _publish_reference_markers(self, info=None):
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        ma.markers.append(line_marker(self.raceline[:, :2], "centerline", 0,
                                      (0.5, 0.5, 0.5), stamp=stamp, width=0.03))
        if info is not None:
            nb, nd, nr = info["n_before"], info["n_divert"], info["n_return"]
            div = self.active_waypoints[nb:nb + nd, :2]
            ret = self.active_waypoints[nb + nd:nb + nd + nr, :2]
            ma.markers.append(line_marker(div, "divert", 1, (1.0, 0.5, 0.0),
                                          stamp=stamp, width=0.06))
            ma.markers.append(line_marker(ret, "return", 2, (0.0, 1.0, 0.0),
                                          stamp=stamp, width=0.06))
        self.marker_pub.publish(ma)

    def _publish_opt(self, traj):
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        # always show the full centerline
        ma.markers.append(line_marker(self.raceline[:, :2], "centerline", 0,
                                      (0.5, 0.5, 0.5), stamp=stamp, width=0.03))
        # if diverting, show divert + return from the active array
        if self.maneuver is not None:
            info = self.maneuver
            nb, nd, nr = info["n_before"], info["n_divert"], info["n_return"]
            ma.markers.append(line_marker(self.active_waypoints[nb:nb+nd, :2],
                              "divert", 1, (1.0, 0.5, 0.0), stamp=stamp, width=0.06))
            ma.markers.append(line_marker(self.active_waypoints[nb+nd:nb+nd+nr, :2],
                              "return", 2, (0.0, 1.0, 0.0), stamp=stamp, width=0.06))
        # the optimal rollout
        ma.markers.append(line_marker(traj[:, :2], "opt", 10, (1.0, 0.0, 0.0),
                                      stamp=stamp, width=0.04))
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicMPPI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()