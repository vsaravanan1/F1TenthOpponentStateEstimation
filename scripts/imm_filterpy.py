#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from filterpy.kalman import KalmanFilter, IMMEstimator
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose

class IMMNode(Node):
    def __init__(self):
        super().__init__('imm_predictor')
        
        # Initial dt - will be updated dynamically in callback based on rate of received messages
        self.dt = 0.05
        self.prev_deg = 0.00
        self.prev_w = 0.0
        
        # Create the kalman filters
        # State vector: [x, y, vx, vy, ax, ay]
        kf_cv = self.create_kf_cv(self.dt)
        kf_ca = self.create_kf_ca(self.dt)
        kf_ct = self.create_kf_ct(self.dt, w=0.5)
        
        # imm filters and initial probabilities of model
        filters = [kf_cv, kf_ca, kf_ct]
        mu = [0.33, 0.33, 0.33]

        # Transition matrix between models
        trans = np.array([[0.98, 0.01, 0.01], 
                          [0.01, 0.98, 0.01], 
                          [0.01, 0.01, 0.98]])
        
    
        self.imm_model = IMMEstimator(filters, mu, trans)
        
    
        self.state_sub = self.create_subscription(
            Float64MultiArray, '/state_vector', self.state_callback, 10)
        self.traj_pub = self.create_publisher(Path, '/imm_path', 10)

    def create_kf_cv(self, dt):
        kf = KalmanFilter(dim_x=6, dim_z=4) # Observe x, y, vx, vy
        kf.F = np.array([
            [1, 0, dt, 0, 0, 0],
            [0, 1, 0, dt, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0], 
            [0, 0, 0, 0, 0, 1]  
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])
        kf.R = np.eye(4) * 0.02
        kf.Q = np.eye(6) * 0.05
        kf.P *= 1.0
        kf.x = np.zeros(6)
        return kf

    def create_kf_ca(self, dt):
        """Creates constant-acceleration Kalman Filter """
        kf = KalmanFilter(dim_x=6, dim_z=4)
        # scale factor of acceleration is very low to minimize chances of predicted trajectory going off the map
        kf.F = np.array([
            [1, 0, dt, 0, 0.001*dt**2, 0],
            [0, 1, 0, dt, 0, 0.001*dt**2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])
        kf.R = np.eye(4) * 0.10
        kf.Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.0001, 0.0001])
        kf.P *= 1.0
        kf.x = np.zeros(6)
        return kf

    def create_kf_ct(self, dt, w):
        """Creates constant-turning Kalman filter with a timestep and an initial value for angular velocity (updated dynamically based on vx and vy)"""
        kf = KalmanFilter(dim_x=6, dim_z=4)
        c, s = np.cos(w*dt), np.sin(w*dt)
        kf.F = np.array([
            [1, 0, dt, 0, 0, 0],
            [0, 1, 0, dt, 0, 0],
            [0, 0, c, -s, 0, 0],
            [0, 0, s, c, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])
        kf.R = np.eye(4) * 0.10
        kf.Q = np.eye(6) * 0.001
        kf.P *= 1.0
        kf.x = np.zeros(6)
        return kf

    def update_filter_matrices(self, dt, vx, vy):
        """Update state transition matrix F accordingly for each model"""
        # CV Model
        self.imm_model.filters[0].F[0, 2] = dt
        self.imm_model.filters[0].F[1, 3] = dt
        
        # CA Model
        self.imm_model.filters[1].F[0, 2] = dt
        self.imm_model.filters[1].F[1, 3] = dt
        self.imm_model.filters[1].F[0, 4] = 0.001 * dt**2
        self.imm_model.filters[1].F[1, 5] = 0.001 * dt**2
        self.imm_model.filters[1].F[2, 4] = dt
        self.imm_model.filters[1].F[3, 5] = dt
        
        # CT Model
        deg = np.degrees(np.arctan2(vy, vx))
        w = np.radians((deg - self.prev_deg)/dt)
        w = np.clip(w, -0.5, 0.5)
        w = 0.35 * self.prev_w + 0.65 * w
        w = np.clip(w, -0.5, 0.5)
        self.prev_deg = deg
        self.prev_w = w
        
        # if deg > 90:
        #     w = 0.5
        # else:
        #     w = -0.5

        c, s = np.cos(w*dt), np.sin(w*dt)
        self.imm_model.filters[2].F[0, 2] = dt
        self.imm_model.filters[2].F[1, 3] = dt
        self.imm_model.filters[2].F[2, 2] = c
        self.imm_model.filters[2].F[2, 3] = -s
        self.imm_model.filters[2].F[3, 2] = s
        self.imm_model.filters[2].F[3, 3] = c

    def state_callback(self, msg):
        # msg: [dt_ms, x, y, vx, vy]
        raw_dt, x, y, vx, vy = msg.data
        dt = raw_dt / 1000.0 if raw_dt > 0 else 0.150
        # handle large lapses in data
        if dt > 0.16:
            return


        self.update_filter_matrices(dt, vx, vy)
        
        self.imm_model.predict()
        z = np.array([x, y, vx, vy])
        self.imm_model.update(z)
        
        pred = self.generate_prediction(steps=15, dt=(dt/15))
        min_x = np.min(pred[:,0])
        max_x = np.max(pred[:,0])
        min_y = np.min(pred[:,1])
        max_y = np.max(pred[:,1])
        distance_squared = (max_x - min_x)*(max_x - min_x) + (max_y - min_y)*(max_y-min_y)

        # filter out trajectories that are too long / go off the map (likely formed due to noisy data or incorrect identification of opponent cluster)
        if distance_squared <= 20:
            self.publish_path(pred.tolist())

    def generate_prediction(self, steps, dt):
        """Generate predicted trajectory by forward-propagation of the best model"""
        curr_state = self.imm_model.x.copy()
        best_idx = np.argmax(self.imm_model.mu)
        F_best = self.imm_model.filters[best_idx].F
        
        prediction = []
        for _ in range(steps):
            curr_state = np.dot(F_best, curr_state)
            prediction.append((curr_state[0], curr_state[1]))
            
        return np.array(prediction)

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

def main(args=None):
    rclpy.init(args=args)
    node = IMMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
