🚜 Autonomous Rover
ROS2 Navigation · SLAM · Obstacle Avoidance · Gazebo Simulation

This repository contains a complete ROS2-based autonomous rover platform supporting:

LiDAR, Ultrasonic, Depth Camera, IMU, GPS

SLAM + Localization + Navigation (Nav2)

Combined obstacle avoidance

Gazebo simulation with agricultural world

RViz visualization

📚 Table of Contents

Requirements

Hardware Components & Installation

LiDAR

Ultrasonic Sensors

Depth Camera

IMU

GPS

Package Overview

rover_obstacle

rover_description

Required Edits (setup.py, CMakeLists)

Build & Environment Setup

Simulation in Gazebo

SLAM, Localization & Navigation

Obstacle Avoidance Pipelines

RViz Visualization

🛠 Requirements

🔝 Back to Top

ROS2 Humble/Jazzy

colcon

Gazebo Classic / Harmonic

Sensor drivers (installed below)

ROS installation commands intentionally not included.

🔧 Hardware Components & Installation

🔝 Back to Top

1. LiDAR
sudo apt install ros-${ROS_DISTRO}-rplidar-ros
sudo apt install ros-${ROS_DISTRO}-laser-filters


Run:

ros2 launch rplidar_ros rplidar.launch.py

2. Ultrasonic Sensors
sudo apt install python3-gpiozero python3-rpi-lgpio


Run:

ros2 run rover_obstacle ultrasonic_reader

3. Depth Camera
sudo apt install ros-${ROS_DISTRO}-realsense2-camera


Run:

ros2 launch realsense2_camera rs_launch.py

4. IMU Sensor
sudo apt install ros-${ROS_DISTRO}-imu-filter-madgwick
sudo apt install ros-${ROS_DISTRO}-robot-localization


Run:

ros2 run imu_filter_madgwick imu_filter_madgwick_node

5. GPS Module
sudo apt install ros-${ROS_DISTRO}-nmea-navsat-driver


Run:

ros2 run nmea_navsat_driver nmea_serial_driver

📦 Package Overview

🔝 Back to Top

rover_obstacle

Python nodes:

1️⃣ obstacle_detector_lidar.py

LiDAR obstacle detection (sector-based).

2️⃣ obstacle_avoidance_lidar.py

Pure LiDAR-based avoidance → /cmd_vel.

3️⃣ ultrasonic_reader.py

Reads HC-SR04 sensors → /ultrasonic_data.

4️⃣ obstacle_avoidance_combined.py

Fusion of LiDAR + ultrasonic → robust avoidance.

5️⃣ slam_explorer.py

Automatic SLAM exploration using SLAM Toolbox.

rover_description

Contains:

URDF model (rover.urdf.xacro)

Gazebo worlds:

empty.world

agricultural_world.world

Launch files:

spawn_rover.launch.py

agricultural.launch.py

localization.launch.py

navigation.launch.py

lidar_convert.launch.py

EKF config → ekf.yaml

📝 Required Edits

🔝 Back to Top

setup.py (rover_obstacle)
entry_points={
    'console_scripts': [
        'obstacle_detector_lidar = rover_obstacle.obstacle_detector_lidar:main',
        'obstacle_avoidance_lidar = rover_obstacle.obstacle_avoidance_lidar:main',
        'ultrasonic_reader = rover_obstacle.ultrasonic_reader:main',
        'obstacle_avoidance_combined = rover_obstacle.obstacle_avoidance_combined:main',
        'slam_explorer = rover_obstacle.slam_explorer:main',
    ],
},

CMakeLists.txt (rover_description)
install(
  DIRECTORY urdf meshes config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

🧱 Build & Environment Setup

🔝 Back to Top

Build the workspace
colcon build --packages-select rover_description rover_obstacle
source install/setup.bash

Export ROS networking settings
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0

🌍 Simulation in Gazebo

🔝 Back to Top

Empty world
ros2 launch rover_description spawn_rover.launch.py world:=empty

Agricultural world
ros2 launch rover_description agricultural.launch.py

🧭 SLAM, Localization & Navigation

🔝 Back to Top

1. SLAM Mapping
ros2 launch rover_description localization.launch.py
ros2 launch slam_toolbox online_async_launch.py


Save map:

ros2 run nav2_map_server map_saver_cli -f ~/map

2. Localization with Saved Map
ros2 launch rover_description localization.launch.py map:=/path/to/map.yaml

3. Navigation (Nav2)
ros2 launch rover_description navigation.launch.py

🧩 Obstacle Avoidance Pipelines

🔝 Back to Top

LiDAR-only avoidance
ros2 run rover_obstacle obstacle_avoidance_lidar

Ultrasonic-only
ros2 run rover_obstacle ultrasonic_reader

Combined LiDAR + Ultrasonic (Recommended)
ros2 run rover_obstacle obstacle_avoidance_combined

SLAM Explorer
ros2 run rover_obstacle slam_explorer

🖥️ RViz Visualization

🔝 Back to Top

Use RViz to visualize robot state, mapping, costmaps, and sensors.

Launch:

rviz2

✔ Recommended RViz Displays
Robot Model
Add → RobotModel
Topic: /robot_description

TF Tree
Add → TF

LaserScan
Add → LaserScan
Topic: /scan

Ultrasonic Range
Add → Range
Topic: /ultrasonic_data

SLAM Map
Add → Map
Topic: /map

Odometry
Add → Odometry
Topic: /odom

Navigation Stack

Global Costmap → /global_costmap/costmap

Local Costmap → /local_costmap/costmap

Global Plan → /plan

Local Plan → /local_plan

IMU
Add → Imu
Topic: /imu/data

Depth Camera (Optional)

PointCloud2 → /camera/depth/color/points

Image → /camera/color/image_raw

⭐ RViz Recommended Settings

Fixed Frame: map (or base_link for testing)

Grid: Enabled

Axes: Enabled

Frame rate: 30 FPS
