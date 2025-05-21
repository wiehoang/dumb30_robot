## Project Overview

Dumb30 is a robotics project focused on learning and implementing robot development from the ground up using ROS2.

## Goal
The primary goal is to build a simple robot capable of Simultaneous Localization and Mapping (SLAM) for autonomous navigation using LiDAR data.

Other goals include:

*   To gain hands-on experience in building a robot from scratch.
*   To learn and apply the concepts of ROS2.
*   To develop and test robot functionalities in a simulated environment before physical deployment.
*   To understand the relation between low-level hardware control and high-level processing.

## Hardware

The robot is built using these core components:

*   **Raspberry Pi 5 (8GB version)**: A board with an active cooler and an NVMe SSD to enhance performance for running the ROS2 environment, processing sensor data, and executing navigation algorithms.
*   **RPLiDAR A1**: A low-cost, indoor 2D LiDAR providing 360-degree scan data, connected to the Pi 5 via USB.
*   **Motor Control:** Microcontroller with a motor driver
    *   **Arduino**: Responsible for the low-level control of motors.
    *   **L298N**: A driver board that interfaces between the Arduino and the motors.
*   **Wheels:** 2x encoded wheels.
*   **Power Bank**: Main energy source for the Raspberry Pi 5 and LiDAR.
*   **LiPo Battery**: Energy source for the Arduino and motors.

## Features
*   **Simulation and Modeling:**
    *   The physical structure of Dumb30 (its links, joints, sensors, and their relationships) is defined by URDF with Xacro files.
    *   This model is used by both Gazebo for simulation (simulate robot's physics, sensor behaviors) and RViz2 for visualization (visualize the robot's states, pose, sensor data, generated SLAM map).
*   **Odometry:**
    *   The Raspberry Pi sends velocity commands (via a ROS2 node, `geometry_msgs/Twist` messages on a `/cmd_vel` topic) to the Arduino.
    *   The Arduino reads encoder data directly from the motors, then calculates the robot's linear and angular movement.
    *   The Arduino sends that odometry information back to the Raspberry Pi. A ROS2 node on the Pi then publishes as `nav_msgs/Odometry` messages.
*   **SLAM Navigation:**
    *   LiDAR data is the primary input for SLAM. SLAM algorithms (running as ROS2 nodes on the Raspberry Pi using `slam_toolbox`) use LiDAR and odometry data.
    *   These algorithms simultaneously estimate the robot's pose (position and orientation) and build a map for the environment.
    *   RViz2 (a visualization tool in ROS2) will visualize the LiDAR scans, the map being built, and the robot's estimated pose.

## System Architecture
To Be Updated
