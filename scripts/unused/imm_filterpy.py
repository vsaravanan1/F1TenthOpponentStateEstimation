#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from filterpy.kalman import KalmanFilter, IMMEstimator
from std_msgs.msg import Float64MultiArray, String, Float64, ColorRGBA
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from rclpy.time import Time, Duration
from visualization_msgs.msg import Marker
import csv
from copy import copy, deepcopy


class IMMNode(Node):
    def __init__(self):
        super().__init__('imm_predictor')
        
        # Initial dt - will be updated dynamically in callback based on rate of received messages
        self.dt = 0.050
        self.prev_deg = 0.00
        self.prev_w = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.first_callback = True
        self.last_odom_pub_time = self.get_clock().now()
        self.filter_counts = np.zeros(3)
        self.num_callbacks = 0
        self.frequencies = np.empty(3, dtype=np.float64)
        self.global_raceline_poses = np.zeros((1000, 2))
        self.raceline_updated = False
        self.num_timesteps = 3

        # Create the kalman filters
        # State vector: [x, vx, ax, y, vy, ay]
        kf_cv = self.create_kf_cv(self.dt)
        kf_ca = self.create_kf_ca(self.dt)
        kf_ct = self.create_kf_ct(self.dt, w=0.5)
        
        # imm filters and initial probabilities of model
        filters = [kf_cv, kf_ca, kf_ct]
        mu = [0.33, 0.33, 0.34]

        # Transition matrix between models
        trans = np.array([[0.98, 0.01, 0.01],   
                          [0.25, 0.5, 0.25], 
                          [0.01, 0.01, 0.98]])
        
    
        self.imm_model = IMMEstimator(filters, mu, trans)
        self.imm_model_inactive =  False
        self.imm_active_pub = self.create_publisher(String, '/imm_active', 10)
        self.imm_active_cb_timer = self.create_timer(0.020, self.imm_active_cb, None, self.get_clock())

    
        self.state_sub = self.create_subscription(Float64MultiArray, '/state_vector', self.state_callback, 10)

        self.traj_pub = self.create_publisher(Path, '/imm_path', 10)
        self.points_marker_pub = self.create_publisher(Marker, '/opp_points_marker', 10)
        self.wait_count = 0
        self.chosen_filter_pub = self.create_publisher(String, '/chosen_filter', 10)

        self.last_state_cb_time = self.get_clock().now()
        self.publish_interval = 0.005
        self.inactive_state_cb_timer = self.create_timer(0.050, self.inactive_state_cb, None, self.get_clock())
        self.raceline_sub = self.create_subscription(Path, '/global_raceline', self.global_raceline_cb, 10)
    

    def imm_active_cb(self):
        if self.imm_model_inactive:
            self.imm_active_pub.publish(String(data="False"))
        else:
            self.imm_active_pub.publish(String(data="True"))

    def inactive_state_cb(self):
        if self.get_clock().now() - self.last_state_cb_time >= Duration(seconds=0, nanoseconds=300e6):
            self.imm_model_inactive = True
            self.imm_model.predict()
            # [x, vx, ax, y, vy, ay]
            # Clamp acceleration to reasonable bounds just in case the traj gen goes off map
            self.imm_model.x[2] = np.clip(self.imm_model.x[2], -3.0, 3.0)  # ax
            self.imm_model.x[5] = np.clip(self.imm_model.x[5], -3.0, 3.0)  # ay
            
            # add decay for acceleration
            accel_decay = 0.9
            self.imm_model.x[2] *= accel_decay
            self.imm_model.x[5] *= accel_decay
            
            # Also clamp velocity if needed
            self.imm_model.x[1] = np.clip(self.imm_model.x[1], -10.0, 10.0)  # vx
            self.imm_model.x[4] = np.clip(self.imm_model.x[4], -10.0, 10.0)  # vy
            

            if self.raceline_updated:
                z_invisible = [self.imm_model.x[0], self.imm_model.x[3]]
                closest_idx, closest_rl_point = self.find_closest_point_raceline(z_invisible)

                original_R = [f.R.copy() for f in self.imm_model.filters]
            
                for f in self.imm_model.filters:
                    f.R = np.eye(2) * (5.0)

                self.imm_model.update(closest_rl_point)
                
                for i, f in enumerate(self.imm_model.filters):
                    f.R = original_R[i]

            pred, future_state, future_cov = self.generate_prediction_with_covariance(steps=20, dt=(self.dt/10))
            self.publish_markers(future_state, future_cov)

            self.publish_path(pred.tolist())
            self.get_logger().info("Published!")
                
    def global_raceline_cb(self, msg : Path):
        if not self.raceline_updated:
            for i, pose in enumerate(msg.poses):
                self.global_raceline_poses[i, 0] = pose.pose.position.x
                self.global_raceline_poses[i, 1] = pose.pose.position.y
            self.raceline_updated = True
    
    def find_closest_point_raceline(self, z):
        displacements = (self.global_raceline_poses - np.expand_dims(z, axis=0))
        disp_norms = np.linalg.norm(displacements, ord=2, axis=1)
        self.get_logger().info(f"{disp_norms.shape}")
        idx = np.argmin(disp_norms)
        return idx, self.global_raceline_poses[idx]
    

    def create_kf_cv(self, dt):
        kf = KalmanFilter(dim_x=6, dim_z=2) # Observe x, y
    
        kf.F = np.array([ # change to use vy instead of acceleraton
            [1, dt, 0,  0,  0,  0],
            [0,  1, 0,  0,  0,  0], 
            [0,  0, 1,  0,  0,  0], 
            [0,  0, 0,  1, dt,  0],
            [0,  0, 0,  0,  1,  0],  
            [0,  0, 0,  0,  0,  1],  
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
        ])
        
        kf.R = np.eye(2) * 0.05

        q_pos = 0.5
        q_vel = 1.0
        q_accel = 1.0

        kf.Q = np.diag([
            q_pos, q_vel, q_accel,
            q_pos, q_vel, q_accel
        ])

        kf.P = np.eye(6) * 1.0
        kf.x = np.zeros(6)
        return kf

    def create_kf_ca(self, dt):
        """Creates constant-acceleration Kalman Filter """
        kf = KalmanFilter(dim_x=6, dim_z=2)
        kf.F = np.array([
            [1, dt, 0.5 * dt**2, 0, 0, 0],
            [0, 1, dt, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, dt, 0.5 * dt**2],
            [0, 0, 0, 0, 1, dt],
            [0, 0, 0, 0, 0, 1]
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])
        # kf.R = np.eye(4) * 0.10
        # kf.Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.1, 0.1])
        # kf.P *= 1.0
        # kf.x = np.zeros(6)
        kf.R = np.eye(2) * 0.05

        q_pos = 1.0
        q_vel = 3.0
        q_accel = 5.0

        kf.Q = np.diag([
            q_pos, q_vel, q_accel,
            q_pos, q_vel, q_accel
        ])

        kf.P = np.eye(6) * 2.0
        kf.x = np.zeros(6)
        
        return kf

    def create_kf_ct(self, dt, w):
        """Creates constant-turning Kalman filter with a timestep and an initial value for angular velocity (updated dynamically based on vx and vy)"""
        kf = KalmanFilter(dim_x=6, dim_z=2)
        c, s = np.cos(w*dt), np.sin(w*dt)
        if w == 0:
            w = 0.01

        kf.F = np.array([
            [1, s/w, (1 - c)/(w**2), 0, 0, 0],
            [0, c,   s/w, 0, 0, 0],
            [0, -1 * w * s, c, 0, 0, 0],
            [0, 0, 0, 1, s/w, (1 - c)/(w**2)],
            [0, 0, 0, 0, c, s/w],
            [0, 0, 0, 0, -1 * w * s, c]
        ])


        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])

        q_pos = 0.5
        q_vel = 1.0
        q_accel = 1.0

        kf.Q = np.diag([
            q_pos, q_vel, q_accel,
            q_pos, q_vel, q_accel
        ])

        kf.R = np.eye(2) * 0.05
        kf.P = np.eye(6) * 1.0
        kf.x = np.zeros(6)
        return kf

    def reset_filter_matrices(self):
        self.imm_model.filters[0] = self.create_kf_cv(self.dt)
        self.imm_model.filters[1] = self.create_kf_ca(self.dt)
        self.imm_model.filters[2] = self.create_kf_ct(self.dt, w=0.5)

    def update_filter_matrices(self, dt, w):
        """Update state transition matrix F accordingly for each model"""
        wdt = w * dt
        c = np.cos(wdt)
        s = np.sin(wdt)

        if abs(w) < 0.001:
            # if w is too small, use small angle approximation
            sw = dt 
            lhs = 1/2 * dt**2
        else: 
            sw = s/w
            lhs = (1 - c) / (w**2)

        f_ct = np.array([
            [1, sw, lhs, 0,    0,      0],
            [0, c,   sw, 0,    0,      0],
            [0, -w*s, c, 0,    0,      0],
            [0, 0,    0, 1,    sw,    lhs],
            [0, 0,    0, 0,    c,      sw],
            [0, 0,    0, 0,    -w*s,   c]
            ])
        self.imm_model.filters[2].F = f_ct

    def state_callback(self, msg):
        self.last_state_cb_time = self.get_clock().now()
        self.imm_model_inactive = False
        # msg: [dt_ms, x, y, vx, vy]
        raw_dt, x, y, vx, vy = msg.data
        dt = raw_dt / 1000.0 if raw_dt > 0 else self.dt
        # handle large lapses in data
        # if dt > 1.00:
        #     self.reset_filter_matrices()
        #     return

        if self.first_callback:
            self.first_callback = False
            for kf in self.imm_model.filters:
                kf.x[0] = x
                kf.x[3] = y
                kf.x[1] = vx
                kf.x[4] = vy
            self.imm_model.x = self.imm_model.mu @ [f.x for f in self.imm_model.filters]
            return


        cross_product = np.cross(np.array([self.imm_model.x[1], self.imm_model.x[4], 0]), np.array([self.imm_model.x[2], self.imm_model.x[5], 0]))
        vel_mag = np.sqrt(self.imm_model.x[1]**2 + self.imm_model.x[4]**2)
        accel_mag = np.sqrt(self.imm_model.x[2]**2 + self.imm_model.x[5]**2)

        # state vector is (x, vx, ax, y, vy, ay)
        if vel_mag < 0.01:
            w = 0.0
        else:
            w = np.clip(accel_mag/vel_mag, -0.3, 0.3)
        
        w *= np.sign(cross_product[2])
            
            
        self.update_filter_matrices(dt, w)
        
        z = np.array([x, y])
        self.imm_model.predict()
        self.imm_model.update(z)

        # [x, vx, ax, y, vy, ay]
        # Clamp acceleration to reasonable bounds just in case the taj gen goes off map
        self.imm_model.x[2] = np.clip(self.imm_model.x[2], -3.0, 3.0)  # ax
        self.imm_model.x[5] = np.clip(self.imm_model.x[5], -3.0, 3.0)  # ay
        
        accel_decay = 0.98
        if self.imm_model.x[2]**2 + self.imm_model.x[5]**2 > (2.25**2):
            self.imm_model.x[2] *= accel_decay
            self.imm_model.x[5] *= accel_decay

        # Also clamp velocity if needed
        self.imm_model.x[1] = np.clip(self.imm_model.x[1], -10.0, 10.0)  # vx
        self.imm_model.x[4] = np.clip(self.imm_model.x[4], -10.0, 10.0)  # vy


        # reducing prediction steps to less lag 
        pred, future_state, future_cov = self.generate_prediction_with_covariance(steps=20, dt=(dt/10))
        self.publish_markers(future_state, future_cov)
        self.publish_path(pred.tolist())
        self.get_logger().info("Published!")

    def publish_markers(self, state, cov):
        points = np.random.multivariate_normal(state, cov, (100,)).tolist()
        points_marker = Marker()
        
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        points_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.header.frame_id = "map"
        points_marker.id = 8
        points_marker.points = []
        for point in points:
            marked_point = Point()
            marked_point.x = point[0]
            marked_point.y = point[1]
            marked_point.z = 0.0
            points_marker.points.append(marked_point)
        
        self.points_marker_pub.publish(points_marker)
        
    def generate_prediction_with_covariance(self, steps, dt):
        """Generate predicted trajectory by forward-propagation of the best model"""
        curr_state = self.imm_model.x.copy()
        curr_cov = self.imm_model.P.copy()
        best_idx = np.argmax(self.imm_model.mu)
        filter_names = ['cv', 'ca', 'ct']
        chosen_filter = filter_names[best_idx]
        print(chosen_filter)
        filter_info = String()
        filter_info.data = chosen_filter
        self.chosen_filter_pub.publish(filter_info)
        F_avg = np.zeros_like(self.imm_model.filters[0].F)
        for i in range(3):
            F_avg += self.imm_model.mu[i] * self.imm_model.filters[i].F


        prediction = np.zeros((steps, 2))
        for i in range(steps):
            curr_state = np.dot(F_avg, curr_state)
            curr_cov = np.dot(np.dot(F_avg, curr_cov), F_avg.T)
            prediction[i] = [curr_state[0], curr_state[3]]
        
        curr_state_xy = np.array([curr_state[0], curr_state[3]])
        curr_cov_xy = np.empty((2, 2))
        curr_cov_xy[0, 0] = curr_cov[0, 0]
        curr_cov_xy[1, 1] = curr_cov[3, 3]
        curr_cov_xy[0, 1] = curr_cov[0, 3]
        curr_cov_xy[1, 0] = curr_cov[3, 0]

        return np.array(prediction), curr_state_xy, curr_cov_xy
    
    def publish_path(self, points):
        """Publish path message to indicate opponent trajectory"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for pt in points:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
            
        self.traj_pub.publish(path_msg)

    def publish_model_freqs(self):
        filter_names = ["cv", "ca", "ct"]
        chosen_filter_idx = np.argmax(self.imm_model.mu)
        self.filter_counts[chosen_filter_idx] += 1
        chosen_filter = filter_names[chosen_filter]
        self.num_callbacks += 1
        if self.num_callbacks >= 1:
            self.frequencies = self.filter_counts/self.num_callbacks
        if self.num_callbacks == 500:
            self.filter_counts = np.zeros(3)
            self.num_callbacks = 0
            return
        with open("frequencies.csv", "w") as csv_file:
            csvwriter = csv.writer(csv_file)
            csvwriter.writerow(self.frequencies.tolist())

def main(args=None):
    rclpy.init(args=args)
    node = IMMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
