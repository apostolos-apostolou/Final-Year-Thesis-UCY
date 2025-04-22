# Autonomous Navigation and Localization Package

This package enables **autonomous navigation** and **localization** for both real and simulated robots using **Nav2** and pre-existing maps. It includes modifications to the Nav2 `bringup` package to integrate seamlessly with the custom robot implementation.

---

## Features

- **Localization**: Uses **Adaptive Monte Carlo Localization (AMCL)** to localize the robot within a pre-existing map.
- **Navigation**: Creates a cost map using map and LiDAR data, enabling the robot to navigate autonomously to specified locations.
  - Planner server: A* Algorithm
  - Controller server: MMPI Controller
- **Real vs Simulation**: Separate launch files for real and simulated robots.
- **Twist Stamper Node**: Converts `cmd_vel` to stamped `cmd_vel` for compatibility with ROS2 Jazzy and `robot_control_pkg`.


## Usage 
### 1. **Start the Localization Launch File**  
   ```ros2 launch nav_pkg localization_launch_real.py map:=./mapname.yaml```
   
   ```ros2 launch nav_pkg localization_launch_sim.py map:=./mapname.yaml```
   
- After launching, manually type “map” as fixed frame in RVIZ.
- Set pose estimation (2D Pose Estimate) if inital pose is not configured in the parameters file.
- Set Durability Policy to: Transient Local for the map.
  
### 2. **Start the Navigation Launch File**  
   ```ros2 launch nav_pkg navigation_launch_real.py map_subscribe_transient_local:=true```
   
   ```ros2 launch nav_pkg navigation_launch_sim.py map_subscribe_transient_local:=true```
   
- After launching, you can visualize local and global costmap in RVIZ as well as the robot's footprint.
- Give a 2D Pose Goal and the robot will move towards the goal.

## Dependencies

Make sure the following dependencies are installed:

```bash
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
twist stamper: https://github.com/joshnewans/twist_stamper 
