import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

os.environ['ROS_LOCALHOST_ONLY'] = '1' 
os.environ['ROS_DOMAIN_ID'] = '0'

def generate_launch_description():

    # 1. SETUP PATHS
    pkg_name = 'rover_description'
    file_subpath = 'urdf/rover.urdf.xacro'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = Command(['xacro ', xacro_file])

    # 2. LAUNCH GAZEBO (The Standard Way)
    # This automatically loads libgazebo_ros_init.so and libgazebo_ros_factory.so
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 3. ROBOT STATE PUBLISHER
    # IMPORTANT: use_sim_time=True is required for SLAM to work!
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # 4. SPAWN THE ROBOT
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'rover'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])