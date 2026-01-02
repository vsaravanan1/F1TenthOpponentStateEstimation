from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lidar_processing",
            executable="bounding_box",
            name="bounding_box"
        ),
        Node(
            package="lidar_processing",
            executable="scan_to_cloud",
            name="scan_to_cloud"
        ),
        Node(
            package="lidar_processing",
            executable="clustering",
            name="cluster"
        ),
        Node(
            package="lidar_processing",
            name="imm",
            executable="imm_filterpy.py"
        ),
    ])

