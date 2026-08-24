#!/usr/bin/env python3

# subscriptions: /imm_path, /centerline
# published: /overtake_spline
# spline to rejoin the global raceline

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import scipy
import numpy as np
from racetrack_utilities.racetrack_utilities import RacetrackUtilities as RUtil


class FrenetInterceptor(Node):
    # class variable
    rutil = RUtil("Spielberg_map.csv")
    CAR_WIDTH = 0.4

    def __init__(self):
        super().__init__('frenet_interceptor')
        imm_path_sub = self.create_subscription(Path, '/imm_path', self.imm_path_cb)

    def imm_path_cb(self, msg : Path):
        imm_poses= msg.poses 
        imm_pose_array = np.zeros((len(imm_poses), 2))
        for i, pose in enumerate(imm_poses):
            imm_pose_array[i, 0] = pose.position.x
            imm_pose_array[i, 1] = pose.position.y

    @classmethod
    def plan_overtake(cls, ego_pose : np.ndarray, opp_pose_array : np.ndarray):
        opp_s, opp_d = cls.rutil.convert_to_frenet(*opp_pose_array[0])
        ego_s, ego_d = cls.rutil.convert_to_frenet(ego_pose)

        overtake_valid = 2 <= (opp_s - ego_s) <= 10
            

        opp_pose_array_frenet = np.copy(opp_pose_array)
        for i, pose in enumerate(opp_pose_array):
            opp_pose_array_frenet[i] = cls.rutil.convert_to_frenet(*opp_pose_array[i])


        control_points = np.zeros(5, 2)
        control_points[0] = np.array([ego_s, ego_d])


        # calculate limits of racetrack at that specific value of s
        lw, rw = cls.rutil.get_bounds_frenet(opp_s)
        lw_clear = lw - cls.CAR_WIDTH/2.0
        rw_clear = rw - cls.CAR_WIDTH/2.0

        opp_d_left = opp_d + cls.CAR_WIDTH
        opp_d_right = opp_d - cls.CAR_WIDTH

        right_clearance = rw_clear + opp_d_right
        left_clearance = lw_clear - opp_d_left

        if right_clearance > 0 and left_clearance > 0:
            if right_clearance > left_clearance:
                control_point_center = [opp_s, opp_d_right - 2/3 * right_clearance]
            else:
                control_point_center = [opp_s, opp_d_left + 2/3 * left_clearance]

        

        # if we can do an overtake, let's get the guide control points
        if overtake_valid:
            pass
            
            
            

        


    def plan_overtake(self, direction : str,  guide_control_points : np.ndarray):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = FrenetInterceptor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Execution Interrupted. Exiting.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    pass