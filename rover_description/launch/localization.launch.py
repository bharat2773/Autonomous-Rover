import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'rover_description'
    config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. Local EKF (Odom -> Base_Link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_local',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),

        # 2. Global EKF (Map -> Odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_global',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),

        # 3. NavSat Transform (GPS Lat/Long -> Odom X/Y)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/global') # Listens to Global EKF
            ]
        )
    ])