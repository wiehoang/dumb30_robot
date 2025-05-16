## Project Overview
Dumb30 is a robotics project focused on learning and implementing robot development from the ground up using ROS2.

## Goal
The primary goal is to build a simple robot capable of Simultaneous Localization and Mapping (SLAM) for autonomous navigation using LiDAR data.

Other are:
* To gain hands-on experience in building a robot from scratch.
* To learn and apply the concepts of ROS2.
* To understand the relation between low-level hardware control and high-level processing.

## Hardware
The robot is built using these core components:

* **Raspberry Pi 5 (8gb version)**: a board with an active cooler, a NVMe SSD to enhance performance for running the ROS2 environment, processing sensor data, and executing navigation algorithms.
* **RPLiDAR A1**: a low-cost, indoor 2D LiDAR provides 360-degree scan data, connected to Pi 5 via USB.
* **Motor Control:** microcontroller with a motor driver
  * Arduino: responsible for the low-level control of motors.
  * L298N: A driver board interfaces between the Arduino and the motors.
* **Wheel:** 2 x encoded wheels
* **Power Bank**: main energy source for Pi 5 and LiDar
* **LiPo Battery**: energy source for Arduino and motors

## Features

* **Odometry:**
  * The Raspberry Pi sends velocity commands (via a ROS2 node as geometry_msgs/Twist messages on a /cmd_vel topic) to the Arduino.
  * The Arduino reads encoder data from the wheels to calculate the distance traveled by each wheel.
  * The Arduino sends odometry information back to the Raspberry Pi. A ROS2 node on the Pi then publishes as nav_msgs/Odometry messages to do SLAM.

* **SLAM Navigation:**
  * The LiDaR data is the primary input data for SLAM.
  * A ROS2 nodes (using slam_toolbox package) combine LiDAR and odometry data to estimate the robot's pose (position & orientation) within the environment and build a map
  * Rviz2 (a visualization tool in ROS2) will visualize the LiDAR scans, the map being built, and the robot's estimated pose.

## System Architecture
TBU
