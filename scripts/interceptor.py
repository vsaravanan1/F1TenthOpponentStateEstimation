#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize_scalar
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math

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
        self.ego_state_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_state_callback, 10)

        self.interceptor_pub = self.create_publisher(Path, '/interceptor_spline', 10)

        self.intercept_marker_pub = self.create_publisher(Marker, '/intercept_point_marker', 10)

        self.lidar_scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_scan_callback, 10)

        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_vx = 0.0
        self.ego_vy = 0.0
        self.max_ego_speed = 5.0
        self.max_ego_accel = 3.0
        self.length_of_the_car = 0.5
        self.opp_path = None

        self.dt = 0.05  # Time step between prediction points

        self.spline_resolution = 20 # num of pts for spline, make bigger for tighter curves and more accurate splines
        
        self.distances_by_angle = np.zeros(1080)

        self.get_logger().info("-----------started Interceptor node----------")

    def lidar_scan_callback(self, msg):
        self.processed_lidar = []
        rmin = max(0.0, msg.range_min)
        rmax = msg.range_max if math.isfinite(msg.range_max) else 4.0
        cap = min(4.0, rmax)
        for r in msg.ranges:
            if not math.isfinite(r) or r <= rmin:
                self.processed_lidar.append(0.0)
            else:
                self.processed_lidar.append(min(float(r), cap))
        self.distances_by_angle = np.array(self.processed_lidar)



    def ego_state_callback(self, msg):

        # gets the ego vehicle state
        
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = msg.twist.twist.linear.x
        self.ego_vy = msg.twist.twist.linear.y

        self.get_logger().info(f"Ego updated: ({self.ego_x:.2f}, {self.ego_y:.2f})")


    def imm_path_callback(self, path_msg):
        # here get the predicted path of oppornenet vehcile 
        print("GOT IMM PATH FOR INTERCEPTOR")
        if( len(path_msg.poses) == 0):
            return
        
        opponent_trajectory = np.array([
            [pose.pose.position.x, pose.pose.position.y] 
            for pose in path_msg.poses
        ]) # In a numpy array

        self.opp_path = opponent_trajectory

        # Find the best intercept point

        intercept_point, idx = self.find_optimal_intercept(opponent_trajectory)

        if intercept_point is not None:
            self.get_logger().info("[interceptor.py debug] Publishing intercept marker")
            self.publish_intercept_marker(intercept_point)
            self.gen_pub_spline(intercept_point)

        else:
            self.get_logger().warn("[interceptor.py debug] No intercept point found - no path generater")

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
            # return time_diff**2 if time_diff > 0 else abs(time_diff) * 0.1

            # less harsh pentaly if the diffrence is large
            if time_diff > 0:
                return time_diff * 2.0
            else:
                return abs(time_diff) * 0.1
        
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

    def get_waypoints(self, intercept_point):
        ego_pos = np.array([self.ego_x, self.ego_y])
        heading_vect = np.array([self.opp_path[4, 0] - self.opp_path[0, 0], self.opp_path[4, 1] - self.opp_path[0, 1]])
        heading_vect = heading_vect/np.abs(heading_vect)
        heading_angle = np.rad2deg(np.arctan2(heading_vect[1], heading_vect[0]))
        possible_targets = np.array([[np.cos(np.deg2rad(heading_angle+90)), np.sin(np.deg2rad(heading_angle+90))],
                                     [np.cos(np.deg2rad(heading_angle-90)), np.sin(np.deg2rad(heading_angle-90))]])
        possible_waypoints = [intercept_point + possible_targets[0]*0.5, intercept_point + possible_targets[1]*0.5]
        

        

    def gen_pub_spline(self, intercept_point):

        ego_pos = np.array([self.ego_x, self.ego_y])
        print(ego_pos)

        # TO make a spline, make waypoints 
        mid_point = (ego_pos + intercept_point) / 2.0

        control_points = np.array([ego_pos, mid_point, intercept_point])

        #need k +1 points for cubic spline

        if len(control_points) < 4:
            quarter_point = 0.75 * ego_pos + 0.25 * intercept_point
            three_quarter_point = 0.25 * ego_pos + 0.75 * intercept_point
            control_points = np.array([ego_pos, quarter_point, three_quarter_point, intercept_point])
        
        x = control_points[:, 0]
        y = control_points[:, 1]

        try:
            
            # spline will pass through all points - Using SciPy
            tck, u = splprep([x, y], s=0, k=3)
            
            # Generate points along the spline
            u_new = np.linspace(0, 1, self.spline_resolution)
            spline_x, spline_y = splev(u_new, tck)
            
            # Create Path message - To publihs itnerceptor spline 
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for i in range(len(spline_x)):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = path_msg.header.stamp
                pose.pose.position.x = float(spline_x[i])
                pose.pose.position.y = float(spline_y[i])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                
                path_msg.poses.append(pose)
            
            # Publish the spline path
            self.interceptor_pub.publish(path_msg)
            self.get_logger().info(f"Published interceptor spline with {len(path_msg.poses)} points")
            
        except Exception as e:
            self.get_logger().error(f"cannot generate spline: {str(e)}")


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






        


        
