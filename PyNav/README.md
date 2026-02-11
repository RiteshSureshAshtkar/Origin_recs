# PyNav: Autonomous Navigation Stack (ROS 2 / Python)

## Overview
**PyNav** is a modular, high-performance autonomous navigation stack built entirely in Python for ROS 2. Designed for differential drive robots, it implements state-of-the-art path planning and control algorithms to enable robust navigation in complex environments.

The stack features a custom A* global planner for optimal path generation and a Dynamic Window Approach (DWA) local controller for smooth trajectory tracking and dynamic obstacle avoidance. It has been validated in Gazebo simulation environments.

## Key Features
- **Global Path Planning**: A* algorithm implementation for finding the shortest path on a costmap.
- **Local Trajectory Optimization**: DWA controller that generates velocity commands (`cmd_vel`) by optimizing for goal progress, obstacle clearance, and speed.
- **Dynamic Obstacle Avoidance**: Real-time LIDAR-based obstacle detection and avoidance, including tangential driving behaviors and recovery maneuvers (spin, backup, skirt) to escape deadlocks.
- **Modular Architecture**: Cleanly separated planner and controller nodes using ROS 2 Lifecycle management.
- **Visualization**: Full RViz integration for visualizing global paths, local trajectories, and costmaps.

## Prerequisites
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish) or compatible Linux distribution.
- **ROS 2**: Humble Hawksbill or Iron Irwini.
- **Simulator**: Gazebo Classic or Gazebo Ignition.
- **Python**: 3.10+.
- **Dependencies**: `numpy`, `matplotlib`.

## Installation

1.  **Create a ROS 2 Workspace**
    ```bash
    mkdir -p ~/pynav_ws/src
    cd ~/pynav_ws/src
    ```

2.  **Clone the Repository**
    ```bash
    git clone https://github.com/your-username/PyNav.git
    ```

3.  **Install Dependencies**
    Ensure you have the TurtleBot3 simulation packages installed:
    ```bash
    sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description
    sudo apt install python3-numpy
    ```

4.  **Build the Workspace**
    ```bash
    cd ~/pynav_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

## Usage

### 1. Launch Simulation
Start the Gazebo simulation environment (e.g., Cafe World):
```bash
ros2 launch turtlebot3_gazebo cafe_world.launch.py
```

### 2. Launch Navigation Stack
In a new terminal, launch the PyNav stack (Planner, Controller, and Transforms):
```bash
source install/setup.bash
ros2 launch pynav_bringup pynav_navigation.launch.py
```
*Note: This will also launch RViz with the pre-configured navigation view.*

### 3. Set Navigation Goal
Use the **2D Nav Goal** tool in RViz to set a destination for the robot. The global planner will generate a path (green line), and the DWA controller will drive the robot along it, avoiding obstacles dynamically.

## Configuration
Navigation parameters can be tuned in `pynav_bringup/config/pynav.yaml`:
- **Robot Kinematics**: `max_vel`, `max_w`, `max_accel`.
- **DWA Spec**: `goal_cost_gain`, `obstacle_cost_gain`, `speed_cost_gain`.
- **Recovery**: `emergency_brake_distance`.

## License
MIT License
