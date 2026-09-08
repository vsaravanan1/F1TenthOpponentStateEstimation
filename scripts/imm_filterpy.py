#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from filterpy.kalman import KalmanFilter, IMMEstimator
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class IMMNode(Node):
    def __init__(self):
        super().__init__('imm_predictor')

        self.dt = 0.050
        self.first_callback = True
        self.last_odom_pub_time = self.get_clock().now()

        # Create the kalman filters: [x, vx, ax, y, vy, ay]
        kf_cv = self.create_kf_cv(self.dt)
        kf_ca = self.create_kf_ca(self.dt)
        kf_ct = self.create_kf_ct(self.dt, w=0.5)

        filters = [kf_cv, kf_ca, kf_ct]
        mu = [0.33, 0.33, 0.34]

        trans = np.array([
            [0.98, 0.01, 0.01],
            [0.25, 0.50, 0.25],
            [0.01, 0.01, 0.98]
        ])

        self.imm_model = IMMEstimator(filters, mu, trans)

        # Subscriptions & Publishers
        self.odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_callback, 10)
        self.traj_pub = self.create_publisher(Path, '/imm_path_original', 10)

        self.get_logger().info("IMM Predictor Online")

    def create_kf_cv(self, dt):
        kf = KalmanFilter(dim_x=6, dim_z=2)
        kf.F = np.array([
            [1, dt, 0,  0,  0,  0],
            [0,  1, 0,  0,  0,  0],
            [0,  0, 1,  0,  0,  0],
            [0,  0, 0,  1, dt,  0],
            [0,  0, 0,  0,  1,  0],
            [0,  0, 0,  0,  0,  1]
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])
        kf.R = np.eye(2) * 0.05
        kf.Q = np.diag([0.5, 1.0, 1.0, 0.5, 1.0, 1.0])
        kf.P = np.eye(6) * 1.0
        kf.x = np.zeros(6)
        return kf

    def create_kf_ca(self, dt):
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
        kf.R = np.eye(2) * 0.05
        kf.Q = np.diag([1.0, 3.0, 5.0, 1.0, 3.0, 5.0])
        kf.P = np.eye(6) * 2.0
        kf.x = np.zeros(6)
        return kf

    def create_kf_ct(self, dt, w):
        kf = KalmanFilter(dim_x=6, dim_z=2)
        if w == 0:
            w = 0.01
        c, s = np.cos(w*dt), np.sin(w*dt)
        kf.F = np.array([
            [1, s/w, (1 - c)/(w**2), 0, 0, 0],
            [0, c,   s/w, 0, 0, 0],
            [0, -w*s, c, 0, 0, 0],
            [0, 0, 0, 1, s/w, (1 - c)/(w**2)],
            [0, 0, 0, 0, c, s/w],
            [0, 0, 0, 0, -w*s, c]
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])
        kf.Q = np.diag([0.5, 1.0, 1.0, 0.5, 1.0, 1.0])
        kf.R = np.eye(2) * 0.05
        kf.P = np.eye(6) * 1.0
        kf.x = np.zeros(6)
        return kf

    def update_filter_matrices(self, dt, w):
        wdt = w * dt
        c, s = np.cos(wdt), np.sin(wdt)
        if abs(w) < 0.001:
            sw = dt
            lhs = 1/2 * dt**2
        else:
            sw = s/w
            lhs = (1 - c) / (w**2)

        self.imm_model.filters[2].F = np.array([
            [1, sw, lhs, 0, 0, 0],
            [0, c,  sw, 0, 0, 0],
            [0, -w*s, c, 0, 0, 0],
            [0, 0, 0, 1, sw, lhs],
            [0, 0, 0, 0, c, sw],
            [0, 0, 0, 0, -w*s, c]
        ])

    def odom_callback(self, msg : Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_time = self.get_clock().now()
        dt = 0.050

        if (current_time - self.last_odom_pub_time).nanoseconds / 1e9 > dt:
            self.last_odom_pub_time = current_time

            if self.first_callback:
                self.first_callback = False
                for kf in self.imm_model.filters:
                    kf.x[0] = x
                    kf.x[3] = y
                self.imm_model.x = self.imm_model.mu @ [f.x for f in self.imm_model.filters]
                return

            cross_product = np.cross([self.imm_model.x[1], self.imm_model.x[4], 0], [self.imm_model.x[2], self.imm_model.x[5], 0])
            vel_mag = np.sqrt(self.imm_model.x[1]**2 + self.imm_model.x[4]**2)
            accel_mag = np.sqrt(self.imm_model.x[2]**2 + self.imm_model.x[5]**2)

            if vel_mag < 0.01:
                w = 0.0
            else:
                w = np.clip(accel_mag/vel_mag, -0.3, 0.3) * np.sign(cross_product[2])

            self.update_filter_matrices(dt, w)

            self.imm_model.predict()
            self.imm_model.update(np.array([x, y]))

            # Clamping
            self.imm_model.x[2] = np.clip(self.imm_model.x[2], -3.0, 3.0)
            self.imm_model.x[5] = np.clip(self.imm_model.x[5], -3.0, 3.0)
            self.imm_model.x[1] = np.clip(self.imm_model.x[1], -10.0, 10.0)
            self.imm_model.x[4] = np.clip(self.imm_model.x[4], -10.0, 10.0)

            # Generate and publish prediction
            pred = self.generate_prediction(steps=45, dt=(dt/3))
            self.publish_path(pred)

    def generate_prediction(self, steps, dt):
        curr_state = self.imm_model.x.copy()
        F_avg = np.zeros_like(self.imm_model.filters[0].F)
        
        for i in range(3):
            F_avg += self.imm_model.mu[i] * self.imm_model.filters[i].F

        prediction = np.zeros((steps, 2))
        for i in range(steps):
            curr_state = np.dot(F_avg, curr_state)
            prediction[i] = [curr_state[0], curr_state[3]]
            
        return prediction

    def publish_path(self, points):
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

def main(args=None):
    rclpy.init(args=args)
    node = IMMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()