# Simulation Package

This package is used to simulate the real robot using Gazebo. The control of the robot is managed using the `gz_ros2_control` package.

- **Documentation:** [gz_ros2_control Documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)  
- **Example for Differential Drive Robot:** [gz_ros2_control Demos](https://github.com/ros-controls/gz_ros2_control/tree/jazzy/gz_ros2_control_demos)

---

## Usage

### 1. **Start the Simulation Launch File**  
   This will:  
   - Launch Gazebo.  
   - Run the Robot State Publisher.  
   - Run the Bridge for Lidar Data to ROS2.  
   - Run `ros2_control` spawner scripts.
   - Run Twist Mux so we can use controller while NAV2 runs.

  ```ros2 launch simulation_pkg gaz_sim_launch.py```
### 2. **Start the Joystick Launch File**  
   ```ros2 launch robot_control_pkg joystick.launch.py```

## Dependencies

Make sure the following dependencies are installed:

```bash
sudo apt-get install ros-jazzy-ros-gz
sudo apt install ros-jazzy-ros2-control 
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-gz-ros2-control 
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-twist-mux
