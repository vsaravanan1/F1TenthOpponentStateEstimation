#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from racetrack_utilities.racetrack_utilities import RacetrackUtilities
from scipy.interpolate import BSpline

class OpponentIntentPredictor(Node):
    def __init__(self):
        super().__init__('opponent_intent_predictor')
        
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_callback, 10)
        self.starting_points_sub = self.create_subscription(Marker, '/starting_points', self.starting_points_cb, 10)

        self.threat_splines_pubs = [
            self.create_publisher(MarkerArray, '/predicted_opponent_splines_0', 10),
            self.create_publisher(MarkerArray, '/predicted_opponent_splines_1', 10),
            self.create_publisher(MarkerArray, '/predicted_opponent_splines_2', 10)
        ]

        self.ego_lane_possibilities_pub = self.create_publisher(
            MarkerArray,
            '/ego_lane_possibilities',
            10
        )
        
        self.ego_x = self.ego_y = self.ego_yaw = 0.0
        self.ego_data_received = False
        
        self.path_resolution = 40
        self.last_publish_time = self.get_clock().now()
        self.publish_interval = 0.0
        self.marker_lifetime = 1.0

        self.opponent_velocity = 2.5
        self.ego_velocity = 1.5

        self.rutil = RacetrackUtilities("/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv")
        

    def ego_odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.ego_data_received = True

    def starting_points_cb(self, msg: Marker):
        current_time = self.get_clock().now()

        if (current_time - self.last_publish_time).nanoseconds < self.publish_interval * 1e9:
            return

        starting_points = msg.points

        threat_paths, opponent_time_paths = self.generate_opponent_splines(starting_points)
        ego_paths, ego_time_paths = self.generate_ego_lane_possibilities()

        self.publish_threat_markers(threat_paths)

        # Consider opponent trajectories from ALL 3 starting points.
        best_ego_idx = self.find_best_defense_trajectory(
            opponent_time_paths,
            ego_time_paths
        )

        if best_ego_idx is not None:
            self.publish_ego_lane_possibilities(
                [ego_paths[best_ego_idx]]
            )

        self.last_publish_time = current_time


    def generate_opponent_splines(self, starting_points: list):
        d_traj_candidates = np.linspace(-0.9, 0.9, 5)
        threat_paths = []
        opponent_time_paths = []

        for point in starting_points:
            s_opp, d_opp = self.rutil.convert_to_frenet(point.x, point.y)
            s_ego, d_ego = self.rutil.convert_to_frenet(self.ego_x, self.ego_y)

            point_paths = []
            point_time_paths = []

            for d_traj in d_traj_candidates:
                if abs(d_traj - d_ego) < 0.2:
                    continue 
                    
                P = np.array([
                    [s_opp, d_opp],
                    [s_opp + (1/3)*(s_ego - s_opp), d_traj],
                    [s_opp + (2/3)*(s_ego - s_opp), d_traj],
                    [s_ego, d_traj],
                    [s_ego + (2/3)*(s_ego - s_opp), (d_traj + d_ego)/2.0] 
                ])

                spline_s, spline_d = self.create_bspline(P)

                t_values = np.linspace(0.0, 1.0, self.path_resolution)

                spline_points_cart = [
                    self.rutil.convert_to_cartesian(
                        float(spline_s(t)),
                        float(spline_d(t))
                    )
                    for t in t_values
                ]

                time_path = self.reparameterize_spline(
                    spline_s,
                    spline_d,
                    self.opponent_velocity
                )

                point_paths.append(spline_points_cart)
                point_time_paths.append(time_path)

            threat_paths.append(point_paths)
            opponent_time_paths.append(point_time_paths)

        return threat_paths, opponent_time_paths


    def generate_ego_lane_possibilities(self):
        d_traj_candidates = np.linspace(-0.9, 0.9, 5)
        ego_paths = []
        ego_time_paths = []

        s_ego, d_ego = self.rutil.convert_to_frenet(self.ego_x, self.ego_y)

        for d_traj in d_traj_candidates:
            P = np.array([
                [s_ego, d_ego],
                [s_ego + 0.25, d_ego + 0.5 * (d_traj - d_ego)],
                [s_ego + 0.5, d_traj],
                [s_ego + 1.0, d_traj],
                [s_ego + 1.5, d_traj],
                [s_ego + 2.0, d_traj]
            ])

            spline_s, spline_d = self.create_bspline(P)

            t_values = np.linspace(0.0, 1.0, self.path_resolution)

            spline_points_cart = [
                self.rutil.convert_to_cartesian(
                    float(spline_s(t)),
                    float(spline_d(t))
                )
                for t in t_values
            ]

            time_path = self.reparameterize_spline(
                spline_s,
                spline_d,
                self.ego_velocity
            )

            ego_paths.append(spline_points_cart)
            ego_time_paths.append(time_path)

        return ego_paths, ego_time_paths


    def create_bspline(self, P):
        degree = 3
        n = len(P)

        knots = np.concatenate([
            np.zeros(degree),
            np.linspace(0.0, 1.0, n - degree + 1),
            np.ones(degree)
        ])

        spline_s = BSpline(knots, P[:, 0], degree)
        spline_d = BSpline(knots, P[:, 1], degree)

        return spline_s, spline_d


    def reparameterize_spline(self, spline_s, spline_d, velocity):
        u_values = np.linspace(0.0, 1.0, 1000)

        ds_du = spline_s.derivative()(u_values)
        dd_du = spline_d.derivative()(u_values)

        speed_u = np.sqrt(ds_du**2 + dd_du**2)

        du = np.diff(u_values)

        arc_length = np.concatenate([
            [0.0],
            np.cumsum(
                0.5 * (speed_u[:-1] + speed_u[1:]) * du
            )
        ])

        time_values = arc_length / velocity

        s_values = spline_s(u_values)
        d_values = spline_d(u_values)

        sample_times = np.linspace(
            0.0,
            time_values[-1],
            self.path_resolution
        )

        sample_s = np.interp(
            sample_times,
            time_values,
            s_values
        )

        sample_d = np.interp(
            sample_times,
            time_values,
            d_values
        )

        return np.column_stack((
            sample_times,
            sample_s,
            sample_d
        ))


    def find_best_defense_trajectory(self, opponent_paths, ego_paths):
        """
        Evaluate every ego trajectory against every opponent trajectory
        from all starting points.

        If an ego/opponent trajectory pair does not collide, that pair
        is ignored when calculating the average TTC.

        The ego trajectory with the smallest average TTC is selected.
        """

        if len(opponent_paths) == 0 or len(ego_paths) == 0:
            return None

        average_ttc = []

        for ego_path in ego_paths:
            collision_times = []

            # opponent_paths contains one list of trajectories for
            # each starting point.
            for starting_point_paths in opponent_paths:

                # Check every candidate trajectory for this starting point.
                for opponent_path in starting_point_paths:
                    ttc = self.find_time_to_collision(
                        opponent_path,
                        ego_path
                    )

                    # Ignore trajectory pairs with no collision.
                    if ttc is not None:
                        collision_times.append(ttc)

            # Only average over trajectory pairs that actually collide.
            if len(collision_times) > 0:
                average_ttc.append(np.mean(collision_times))
            else:
                average_ttc.append(np.inf)

        # If none of the ego trajectories collide with any opponent
        # trajectory, there is no defensive trajectory to select.
        if all(np.isinf(value) for value in average_ttc):
            return None

        # Select the ego trajectory with the smallest average TTC.
        return int(np.argmin(average_ttc))


    def find_time_to_collision(self, opponent_path, ego_path):
        if len(opponent_path) == 0 or len(ego_path) == 0:
            return None

        opponent_times = opponent_path[:, 0]
        ego_times = ego_path[:, 0]

        start_time = max(
            opponent_times[0],
            ego_times[0]
        )

        end_time = min(
            opponent_times[-1],
            ego_times[-1]
        )

        if start_time > end_time:
            return None

        collision_check_resolution = 1000

        times = np.linspace(
            start_time,
            end_time,
            collision_check_resolution
        )

        opponent_s = np.interp(
            times,
            opponent_times,
            opponent_path[:, 1]
        )

        opponent_d = np.interp(
            times,
            opponent_times,
            opponent_path[:, 2]
        )

        ego_s = np.interp(
            times,
            ego_times,
            ego_path[:, 1]
        )

        ego_d = np.interp(
            times,
            ego_times,
            ego_path[:, 2]
        )

        collision_mask = (
            (np.abs(opponent_s - ego_s) < 0.3) &
            (np.abs(opponent_d - ego_d) < 0.2)
        )

        collision_indices = np.where(collision_mask)[0]

        if len(collision_indices) == 0:
            return None

        return float(times[collision_indices[0]])


    def publish_threat_markers(self, threat_paths):
        colors = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0)
        ]

        for point_idx, point_paths in enumerate(threat_paths):
            marker_array = MarkerArray()

            r, g, b = colors[point_idx]

            for idx, path in enumerate(point_paths):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"opponent_threats_{point_idx}"
                marker.id = idx
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.05
                
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = 0.8

                marker.lifetime.sec = int(self.marker_lifetime)
                marker.lifetime.nanosec = int(
                    (self.marker_lifetime - int(self.marker_lifetime)) * 1e9
                )
                
                for pt in path:
                    p = Point()
                    p.x = float(pt[0])
                    p.y = float(pt[1])
                    p.z = 0.0
                    marker.points.append(p)
                
                marker_array.markers.append(marker)
            
            self.threat_splines_pubs[point_idx].publish(marker_array)


    def publish_ego_lane_possibilities(self, ego_paths):
        marker_array = MarkerArray()

        for idx, path in enumerate(ego_paths):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ego_lane_possibilities"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int(
                (self.marker_lifetime - int(self.marker_lifetime)) * 1e9
            )

            for pt in path:
                p = Point()
                p.x = float(pt[0])
                p.y = float(pt[1])
                p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.ego_lane_possibilities_pub.publish(marker_array)


    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = OpponentIntentPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()