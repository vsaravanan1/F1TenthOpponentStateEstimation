#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize_scalar
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

"""
This class :
    1. Find the interceptor point (Need to superproject by adding the legnth of the ego vehicle)
    2. Scipy's paramentric cubic spline is used to make a path for ego vehicle
    3. Interceptor spline is the cubic spile 
"""

class IMMInterceptorNode(Node):
    def __init__(self):
        super().__init__('imm_interceptor')

        # need to subscribe to IMM predicted path 
        self.imm_path_sub = self.create_subscription(Path, '/imm_path', self.imm_path_callback, 10)

        # sub to ego vehicle state(need to chage these topcis for car softwarer)
        self.ego_state_sub = self.create_subscription(Float64MultiArray, '/ego_state', self.ego_state_callback, 10)

        self.interpector_pub = self.create_publisher(Path, '/interceptor_spline', 10)

        self.intercept_marker_pub = self.create_publisher(Marker, '/intercept_point_marker', 10)

        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_vx = 0.0
        self.ego_vy = 0.0
        self.max_ego_speed = 5.0
        self.max_ego_accel = 3.0
        self.length_of_the_car = 0.5

        self.dt = 0.05  # Time step between prediction points

        self.get_logger().info("-----------started Interceptor node----------")

    def ego_state_callback(self, msg):

        # gets the ego vehicle state

        self.ego_x = msg.data[0]
        self.ego_y = msg.data[1]
        self.ego_vx = msg.data[2]
        self.ego_vy = msg.data[3]


    def imm_path_callback(self, path_msg):

        # here get the predicted path of oppornenet vehcile 
        print("GOT IMM PATH FOR INTERCEPTOR")
        if( len(path_msg.poses) == 0):
            return
        
        opponent_trajectory = np.array([
            [pose.pose.position.x, pose.pose.position.y] 
            for pose in path_msg.poses
        ]) # In a numpy array

        # Find the best intercept point

        intercept_point, idx = self.find_optimal_intercept(opponent_trajectory)

        if intercept_point is not None:
            self.get_logger().info("[interceptor.py debug] Publishing intercept marker")
            self.publish_intercept_marker(intercept_point)
        else:
            self.get_logger().warn("[interceptor.py debug] No intercept point found")

        # generate interceptor spline

        

    def find_optimal_intercept(self, opponent_trajectory):
        
        ego_pos = np.array([self.ego_x, self.ego_y])
        ego_speed = np.sqrt(self.ego_vx**2 + self.ego_vy**2) 


        def time_difference(t_idx):
            """Cost function: difference between arrival times"""
            idx = int(np.clip(t_idx, 0, len(opponent_trajectory) - 1))
            opp_pos = opponent_trajectory[idx]
            
            distance = np.linalg.norm((opp_pos + self.length_of_the_car) - ego_pos)
            
            # Time for ego to reach this point
            ego_time = distance / ego_speed
            
            # Time for opponent to reach this point
            opp_time = idx * self.dt
            
            time_diff = ego_time - opp_time
            return time_diff**2 if time_diff > 0 else abs(time_diff) * 0.1
        
        # Optimize to find best intercept index
        result = minimize_scalar(
            time_difference,
            bounds=(0, len(opponent_trajectory) - 1),
            method='bounded'
        )
        
        best_idx = int(result.x)
        
        # Check if intercept is feasible
        # if result.fun > 10.0:  # If time difference is too large
        #     return None, -1
        
        intercept_point = opponent_trajectory[best_idx]
        
        self.get_logger().info(
            f"Intercept point found at index {best_idx}/{len(opponent_trajectory)-1}, "
            f"position: ({intercept_point[0]:.2f}, {intercept_point[1]:.2f})"
        )
        
        return intercept_point, best_idx

    def publish_intercept_marker(self, intercept_point):

        """Publish a visualization marker for the intercept point"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "intercept"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(intercept_point[0])
        marker.pose.position.y = float(intercept_point[1])
        marker.pose.position.z = 0.5  
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # Color (bright green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        self.intercept_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = IMMInterceptorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






        


        
