from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="cloud_to_scan",
            output="screen",
            remappings=[
                ("cloud_in", "/lidar/points"),   # <-- CORRECT TOPIC
                ("scan", "/scan")
            ],
            parameters=[{
                "target_frame": "lidar_link",
                "transform_tolerance": 0.01,
                "min_height": -0.3,
                "max_height": 0.3,
                "range_min": 0.12,
                "range_max": 12.0
            }]
        )
    ])
