import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rover = get_package_share_directory('rover_description')  # Change to your package name
    
    # Path to world file
    world_file = os.path.join(pkg_rover, 'worlds', 'agricultural_world.world')
    
    # Path to URDF
    urdf_file = os.path.join(pkg_rover, 'urdf', 'rover.urdf.xacro')  # Change to your URDF name
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
        'use_sim_time': True}]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'rover',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.2'],
        output='screen'
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity
    ])