# IMU Visualizer — ROS2 + ESP32S3 + MPU6050

## 3D F16 Fighter Jet Style IMU Visualization using ROS2 Jazzy

## Hardware
- ESP32S3 Development Board
- MPU6050 IMU Sensor (I2C)
- USB Cable (for Serial communication)

## Software Stack

| Component | Technology |
|-----------|-----------|
| OS | Ubuntu 24.04 LTS |
| Middleware | ROS2 Jazzy Jalisco |
| Firmware IDE | VS Code + ESP-IDF v6.0.2 |
| Firmware Language | C (ESP-IDF) |
| ROS2 Language | Python 3.12 |
| Visualization | RViz2 + URDF |
| Filter | Madgwick Filter |

## ROS2 Packages

| Package | Function |
|---------|----------|
| imu_driver | Reads ESP32 serial, publishes /imu/raw |
| imu_filter | Madgwick filter, publishes /imu/filtered |
| plane_visualizer | TF broadcaster, RViz2 paper plane |
| imu_bringup | Master launch file |

## Quick Start

Step 1 - Build workspace
cd ~/imu_visualizer/ros2_ws
colcon build
source install/setup.bash

Step 2 - Launch
ros2 launch imu_bringup system.launch.py

Step 3 - Visualize
rviz2
