from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lidar_processing",
            executable="scan_to_cloud",
            name="scan_to_cloud"
        ),
        Node(
            package="lidar_processing",
            name="imm",
            executable="imm.py"
        ),
        Node(
            package="lidar_processing",
            name="bounding_box",
            executable="bounding_box"
        ),
        Node(
            package="lidar_processing",
            name='ClusteringNode',
            executable='clustering_rf.py'
        ),
        Node(
            package="lidar_processing",
            name="CenterlinePublisher",
            executable="spielberg_centerline_publisher.py"
        ),
        Node(
            package="lidar_processing",
            name="FrenetStatePublisher",
            executable="frenet_opp_state.py"
        )
    ])

