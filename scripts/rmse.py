#!/usr/bin/env python3

import signal
import sys
import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import String

class IMM_RMSE_EVAL(Node):
    def __init__(self):
        super().__init__('imm_rmse_eval')
        # subscribe to imm path and opponent racecar odometry
        self.imm_sub = self.create_subscription(Path, '/imm_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_callback, 10)
        self.rmse_timer = self.create_timer(0.050, self.rmse_timer_cb, None, self.get_clock())
        self.rmse_pub = self.create_publisher(String, '/rmse_error', 10)
        
        
        # odom callback variables
        self.last_odom_cb_time = self.get_clock().now()
        self.odom_buffer : list[dict] = []
        
        # odom callback constants 
        self.odom_cb_interval : Duration = Duration(seconds = 0, nanoseconds = 5e7)
        self.odom_buf_max_len : int =  50

        # path callback constants
        self.path_step_dt = Duration(seconds = 0, nanoseconds = 150e6)
        self.path_length = 15
        self.pending_paths : list[list] = []
        
        # buffer of errors
        self.rmse_errors = []
        self.rmse_error_buf_max_len = 10
        self.pending_paths = []
    
    def odom_callback(self, msg : Odometry):
        current_time = self.get_clock().now()
        if current_time - self.last_odom_cb_time < self.odom_cb_interval:
            return
        self.last_odom_cb_time = current_time
        if len(self.odom_buffer) >= self.odom_buf_max_len:
            self.odom_buffer.pop(0)
        self.odom_buffer.append({"time": current_time, "pose": msg.pose.pose.position})
        
    def path_callback(self, msg : Path):
        current_time = self.get_clock().now()
        pose_array = []

        for i, pose in enumerate(msg.poses):
            position = pose.pose.position
            pose_array.append({"time": current_time + Duration(nanoseconds=(self.path_step_dt.nanoseconds * i)), "pose": position})
        
        self.pending_paths.append(pose_array)

    def rmse_timer_cb(self):
        if self.pending_paths == [] or self.odom_buffer == []:
            return
        future_paths : list = []
        for path in self.pending_paths:
            latest_odom_time = self.odom_buffer[-1]["time"]
            latest_path_prediction = path[-1]["time"]
            if latest_path_prediction > latest_odom_time:
                future_paths.append(path)
                continue
            else:
                rmse_error : float = 0.0
                for timed_pose in path:
                    time =  timed_pose['time']
                    pose = timed_pose['pose']
                    durations = [((time - odom['time']) if (time > odom['time']) else (odom['time'] - time)) for odom in self.odom_buffer]
                    closest_time_idx = durations.index(min(durations))
                    rmse_error += (pose.x -  self.odom_buffer[closest_time_idx]['pose'].x)**2 + (pose.y -  self.odom_buffer[closest_time_idx]['pose'].y)**2
                    
                rmse_error /= len(path)
                rmse_error = np.sqrt(rmse_error)
                self.rmse_errors.append(rmse_error)
                self.get_logger().info(f"RMSE Error: {rmse_error}")
                rmse_published_msg = String()
                rmse_published_msg.data = f"RMSE Error: {rmse_error}\n"
                self.rmse_pub.publish(rmse_published_msg)
                if len(self.rmse_errors) > self.rmse_error_buf_max_len:
                    self.rmse_errors.pop(0)
        self.pending_paths = future_paths
        
        
def main(args=None):
    rclpy.init(args=args)
    node = IMM_RMSE_EVAL()

    def shutdown():
        node.get_logger().info("\nBag finished or interrupted")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    rclpy.spin(node)

    
if __name__ == "__main__":
    main()
    
    
    
