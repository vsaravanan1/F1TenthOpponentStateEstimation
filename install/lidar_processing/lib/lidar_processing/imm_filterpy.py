#!/usr/bin/env python3
import rclpy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.kalman import IMMEstimator
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

class imm(Node):
    def __init__(self):
        super().__init__('imm')
        kf_cv = create_kf_cv(0, 0, 1, 1, 0.5)
        kf_ca = create_kf_ca(0, 0, 1, 1, 0.5, 0.2, 0.2)
        kf_ct = create_kf_ct(0, 0, 1, 1, 0.5, 0.210)
        filters = [kf_cv, kf_ca, kf_ct]
        mu = [1/3, 1/3, 1/3]
        trans = np.array([[0.7, 0.15, 0.15], [0.15, 0.7, 0.15], [0.15, 0.15, 0.7]])
        self.imm_model = IMMEstimator(filters, mu, trans)
        self.state_vector_sub = self.create_subscription(Float64MultiArray, '/state_vector', self.state_vector_callback, 10)
        self.traj_pub = self.create_publisher(Path, '/imm_path', 10)
        self.prev_vx = 0.0
        self.prev_vy = 0.0

    def state_vector_callback(self, msg : Float64MultiArray):
        dt, x, y, vx, vy = msg.data
        # millisecond to seconds 
        dt /= 1000
        ax = (vx - self.prev_vx)/dt
        ay = (vy - self.prev_vy)/dt
        self.get_logger().info(f"dt: {dt}, x: {x}, y: {y}, vx: {vx}, vy: {vy}, ax: {ax}, ay: {ay}")
        self.get_logger().info(f"mu: {self.imm_model.mu}")
        self.imm_model.predict()
        self.imm_model.update(np.array([x, y, vx, vy, 0.0, 0.0]))
        self.prev_vx = vx
        self.prev_vy = vy
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_msg.poses = []
        imm_path : np.array = generate_traj(self.imm_model, dt, 1.05)
        for i in imm_path:
            x, y, vx, vy = i
            new_pose = PoseStamped()
            new_pose.pose = Pose()
            new_pose.pose.position.x = float(x)
            new_pose.pose.position.y = float(y)
            new_pose.pose.position.z = 0.0
            new_pose.pose.orientation.x = 0.0
            new_pose.pose.orientation.y = 0.0
            new_pose.pose.orientation.z = 0.0
            new_pose.pose.orientation.w = 1.0
            path_msg.poses.append(new_pose)
            
        self.traj_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = imm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# state vector for all 3 filters is x, y, vx, vy, ax, ay

# constant velocity kalman filter
def create_kf_cv(x, y, vx, vy, dt):
    kf_cv = KalmanFilter(dim_x = 6, dim_z = 6)
    kf_cv.x = np.array([x, y, vx, vy, 0, 0])
    kf_cv.P = np.eye(6)
    kf_cv.F = np.array([
        [1, 0, dt, 0, 0, 0],
        [0, 1, 0, dt, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ])
    kf_cv.H = np.eye(6)
    kf_cv.Q = np.eye(6) * 100
#    kf_cv.R = np.diag([4, 4, 9, 9, 2, 2])
    kf_cv.R = np.eye(6)
    kf_cv.B = None
    return kf_cv

# constant turning
def create_kf_ct(x, y, vx, vy, dt, w):
    c = np.cos(w * dt)
    s = np.sin(w * dt)
    kf_ct = KalmanFilter(dim_x = 6, dim_z = 6)
    kf_ct.x = np.array([x, y, vx, vy, 0, 0])
    kf_ct.P = np.eye(6)
    kf_ct.F = np.array([
        [1, 0, dt, 0, 0, 0],
        [0, 1, 0, dt, 0, 0],
        [0, 0, c, -1 * s, 0, 0],
        [0, 0, s, c, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ])
    kf_ct.H = np.eye(6)
    kf_ct.Q = np.eye(6) * 100
#    kf_ct.R = np.diag([4, 4, 9, 9, 2, 2])
    kf_ct.R = np.eye(6)
    kf_ct.B = None
    return kf_ct

# constant linear acceleration
def create_kf_ca(x, y, vx, vy, dt, ax, ay):
    kf_ca = KalmanFilter(dim_x = 6, dim_z = 6)
    kf_ca.x = np.array([x, y, vx, vy, ax, ay])
    kf_ca.P = np.eye(6)
    kf_ca.F = np.array([
        [1, 0, dt, 0, 0.5 * dt * dt, 0],
        [0, 1, 0, dt, 0, 0.5 * dt * dt],
        [0, 0, 1, 0, dt, 0],
        [0, 0, 0, 1, 0, dt],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ])
    kf_ca.H = np.eye(6)
    kf_ca.Q = np.eye(6) * 100
#    kf_ca.R = np.diag([4, 4, 9, 9, 2, 2])
    kf_ca.R = np.eye(6)
    kf_ca.B = None
    return kf_ca


# trajectory generation based on given state vector and set of probabilities
def generate_traj(imm : IMMEstimator, dt, w):
    mu = imm.mu
    x, y, vx, vy, ax, ay = imm.x
    c = np.cos(w * dt)
    s = np.sin(w * dt)

    cv_path = np.array([[0]*4]*5)
    for i in range(5):
        cv_path[i, 0] = x + i * vx * dt
        cv_path[i, 1] = y + i * vy * dt
        cv_path[i, 2] = vx
        cv_path[i, 3] = vy
        
    ca_path = np.array([[0]*4]*5)
    for i in range(5):
        ca_path[i, 0] = x + i * vx * dt + 1/2 * ax * (i * dt)**2
        ca_path[i, 1] = y + i * vy * dt + 1/2 * ay * (i * dt)**2
        ca_path[i, 2] = vx + i * ax * dt
        ca_path[i, 3] = vy + i * ay * dt
    
    ct_path = np.array([[0]*4]*5)
    for i in range(5):
        ct_path[i, 0] = x
        ct_path[i, 1] = y
        ct_path[i, 2] = vx
        ct_path[i, 3] = vy
        
        x = x + vx * dt
        y = y + vy * dt
        vx_old, vy_old = vx, vy
        vx = vx_old * c - vy_old * s
        vy = vx_old * s + vy_old * c

    chosen_index = mu.argmax()
    if chosen_index == 0:
        result_path = cv_path
    elif chosen_index == 1:
        result_path = ca_path
    elif chosen_index == 2:
        result_path = ct_path
    return result_path

if __name__ == "__main__":
    main()     
    
# def main():
#     kf_cv = create_kf_cv(0, 0, -7, 3, 0.5)
#     kf_ca = create_kf_ca(0, 0, 8, -4, 0.5, 2, 0.5)
#     kf_ct = create_kf_ct(0, 0, -2, -3, 0.5, 1.05)
#     filters = [kf_cv, kf_ca, kf_ct]
#     mu = [1/3, 1/3, 1/3]
#     trans = np.array([[0.7, 0.15, 0.15], [0.15, 0.7, 0.15], [0.15, 0.15, 0.7]])
#     imm = IMMEstimator(filters, mu, trans)

#     for i in range(180):
#         np.random.seed(i)
#         vx = 7
#         vy = 3
#         x = vx * i + np.random.randn()*np.sqrt(kf_cv.R[0, 0])
#         y = vy * i + np.random.randn()*np.sqrt(kf_cv.R[1, 1])
#         z = np.array([[x], [y], [vx], [vy], [0], [0]])
#         imm.predict()
#         imm.update(z)
    
#     print(imm.mu)
#     traj = generate_traj(imm, 0.5, 1.05)
#     print(traj)



# if __name__ == "__main__":
#     main()