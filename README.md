# 🚜 Autonomous Rover

## ROS2 Navigation · SLAM · Obstacle Avoidance · Gazebo Simulation

### This repository contains a complete ROS2-based autonomous rover platform supporting:

- 🎯 LiDAR, Ultrasonic, Depth Camera, IMU, GPS
- 🗺️ SLAM + Localization + Navigation (Nav2)
- 🚧 Combined obstacle avoidance
- 🌍 Gazebo simulation with agricultural world
- 📊 RViz visualization

---

# 📚 Table of Contents

- [Requirements](#-requirements)
- [Hardware Components & Installation](#-hardware-components--installation)
  - [LiDAR](#1-lidar)
  - [Ultrasonic Sensors](#2-ultrasonic-sensors)
  - [Depth Camera](#3-depth-camera)
  - [IMU](#4-imu-sensor)
  - [GPS](#5-gps-module)
- [Package Overview](#-package-overview)
- [Required Edits](#-required-edits)
- [Build & Environment Setup](#-build--environment-setup)
- [Simulation in Gazebo](#-simulation-in-gazebo)
- [SLAM, Localization & Navigation](#-slam-localization--navigation)
- [Obstacle Avoidance Pipelines](#-obstacle-avoidance-pipelines)
- [RViz Visualization](#️-rviz-visualization)

---

# 🛠 Requirements

- ROS2 Humble/Jazzy
- colcon
- Gazebo Classic / Harmonic
- Sensor drivers (installed below)

> **Note:** ROS installation commands intentionally not included.

[🔝 Back to Top](#-autonomous-rover)

---

# 🔧 Hardware Components & Installation

### 1. LiDAR

```bash
sudo apt install ros-${ROS_DISTRO}-rplidar-ros
sudo apt install ros-${ROS_DISTRO}-laser-filters
```

**Run:**
```bash
ros2 launch rplidar_ros rplidar.launch.py
```

### 2. Ultrasonic Sensors

```bash
sudo apt install python3-gpiozero python3-rpi-lgpio
```

**Run:**
```bash
ros2 run rover_obstacle ultrasonic_reader
```

### 3. Depth Camera

```bash
sudo apt install ros-${ROS_DISTRO}-realsense2-camera
```

**Run:**
```bash
ros2 launch realsense2_camera rs_launch.py
```

### 4. IMU Sensor

```bash
sudo apt install ros-${ROS_DISTRO}-imu-filter-madgwick
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

**Run:**
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node
```

### 5. GPS Module

```bash
sudo apt install ros-${ROS_DISTRO}-nmea-navsat-driver
```

**Run:**
```bash
ros2 run nmea_navsat_driver nmea_serial_driver
```

[🔝 Back to Top](#-autonomous-rover)

---

## 📦 Package Overview

### `rover_obstacle`

Python nodes:

1. **`obstacle_detector_lidar.py`** - LiDAR obstacle detection (sector-based)
2. **`obstacle_avoidance_lidar.py`** - Pure LiDAR-based avoidance → `/cmd_vel`
3. **`ultrasonic_reader.py`** - Reads HC-SR04 sensors → `/ultrasonic_data`
4. **`obstacle_avoidance_combined.py`** - Fusion of LiDAR + ultrasonic → robust avoidance
5. **`slam_explorer.py`** - Automatic SLAM exploration using SLAM Toolbox

### `rover_description`

Contains:
- **URDF model** (`rover.urdf.xacro`)
- **Gazebo worlds:**
  - `empty.world`
  - `agricultural_world.world`
- **Launch files:**
  - `spawn_rover.launch.py`
  - `agricultural.launch.py`
  - `localization.launch.py`
  - `navigation.launch.py`
  - `lidar_convert.launch.py`
- **EKF config** → `ekf.yaml`

[🔝 Back to Top](#-autonomous-rover)

---

## 📝 Required Edits

### `setup.py` (rover_obstacle)

```python
entry_points={
    'console_scripts': [
        'obstacle_detector_lidar = rover_obstacle.obstacle_detector_lidar:main',
        'obstacle_avoidance_lidar = rover_obstacle.obstacle_avoidance_lidar:main',
        'ultrasonic_reader = rover_obstacle.ultrasonic_reader:main',
        'obstacle_avoidance_combined = rover_obstacle.obstacle_avoidance_combined:main',
        'slam_explorer = rover_obstacle.slam_explorer:main',
    ],
},
```

### `CMakeLists.txt` (rover_description)

```cmake
install(
  DIRECTORY urdf meshes config launch worlds
  DESTINATION share/${PROJECT_NAME}
)
```

[🔝 Back to Top](#-autonomous-rover)

---

## 🧱 Build & Environment Setup

### Build the workspace

```bash
colcon build --packages-select rover_description rover_obstacle
source install/setup.bash
```

### Export ROS networking settings

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
```

[🔝 Back to Top](#-autonomous-rover)

---

## 🌍 Simulation in Gazebo

### Empty world

```bash
ros2 launch rover_description spawn_rover.launch.py world:=empty
```

### Agricultural world

```bash
ros2 launch rover_description agricultural.launch.py
```

[🔝 Back to Top](#-autonomous-rover)

---

## 🧭 SLAM, Localization & Navigation

### 1. SLAM Mapping

```bash
ros2 launch rover_description localization.launch.py
ros2 launch slam_toolbox online_async_launch.py
```

**Save map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 2. Localization with Saved Map

```bash
ros2 launch rover_description localization.launch.py map:=/path/to/map.yaml
```

### 3. Navigation (Nav2)

```bash
ros2 launch rover_description navigation.launch.py
```

[🔝 Back to Top](#-autonomous-rover)

---

## 🧩 Obstacle Avoidance Pipelines

### LiDAR-only avoidance

```bash
ros2 run rover_obstacle obstacle_avoidance_lidar
```

### Ultrasonic-only

```bash
ros2 run rover_obstacle ultrasonic_reader
```

### Combined LiDAR + Ultrasonic (Recommended)

```bash
ros2 run rover_obstacle obstacle_avoidance_combined
```

### SLAM Explorer

```bash
ros2 run rover_obstacle slam_explorer
```

[🔝 Back to Top](#-autonomous-rover)

---

## 🖥️ RViz Visualization

Use RViz to visualize robot state, mapping, costmaps, and sensors.

**Launch:**
```bash
rviz2
```

### ✔ Recommended RViz Displays

| Display | Type | Topic |
|---------|------|-------|
| Robot Model | RobotModel | `/robot_description` |
| TF Tree | TF | - |
| LaserScan | LaserScan | `/scan` |
| Ultrasonic Range | Range | `/ultrasonic_data` |
| SLAM Map | Map | `/map` |
| Odometry | Odometry | `/odom` |
| Global Costmap | Costmap | `/global_costmap/costmap` |
| Local Costmap | Costmap | `/local_costmap/costmap` |
| Global Plan | Path | `/plan` |
| Local Plan | Path | `/local_plan` |
| IMU | Imu | `/imu/data` |
| Depth PointCloud | PointCloud2 | `/camera/depth/color/points` |
| RGB Image | Image | `/camera/color/image_raw` |

### ⭐ RViz Recommended Settings

- **Fixed Frame:** `map` (or `base_link` for testing)
- **Grid:** Enabled
- **Axes:** Enabled
- **Frame rate:** 30 FPS

[🔝 Back to Top](#-autonomous-rover)

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.


**Happy Roving! 🚜✨**
