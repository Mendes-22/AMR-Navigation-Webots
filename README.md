# 🤖 AMR Navigation

This repository contains the implementation of an autonomous mobile robot navigation system developed in **Webots**. The project covers the full navigation pipeline, from raw sensor data acquisition to autonomous path planning and execution.

## 🚀 Core Technologies

* **Mapping (SLAM):** Real-time occupancy grid mapping.
* **Localization:** Extended Kalman Filter (EKF) for sensor fusion.
* **Path Planning:** A* algorithm with dynamic obstacle avoidance.
* **Control:** Proportional navigation for smooth target reaching.

## 🏗️ System Architecture

The project is structured into modular components:

* `kalman_filter.py`: State estimation and sensor fusion for the EKF.
* `path_planner.py`: A* pathfinding engine and trajectory smoothing.
* `SLAM.py`: Controller for SLAM and grid map generation.
* `AMR_main.py`: Controller for path execution and motor control in Webots.

## 🎥 Process Demonstration

#### 📤 Storage Cycle (Load)
The system identifies the box, applies the weight/height-safety rule, and selects the **nearest optimal slot** to minimize travel time.
*![Load Cycle](Media/Load_GIF.gif)*
---
*Project archive for portfolio and personal reference. 🎓*
