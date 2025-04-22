# Robot Control Package

This package is designed to control differential drive robots (such as the "Helicas Robotics" warehouse robot) in conjunction with an Arduino Mega running the `serial_robot_control_arduino` code.

## How the Package Works

- Receives `/cmd_vel` commands (Twist Stamped).
- Utilizes the ROS2 Control Package to:
  - Send the appropriate speed commands to the robot (command interface).
  - Receive motor speed and position (state interfaces) for closed-loop control.
  - Publish the appropriate transforms for the robot (for visualization, mapping and navigation purposes).

**Main Launch File:** `diffbot.launch.py`  
- Enables ROS2 Control node and uses parameters from `diffbot_controllers.yaml`.
- Enables the Robot State Publisher node.
- Enables the Joint State Broadcaster node.
- Enables the Robot Controller Spawner.
- Enables `twist_mux` to allow joystick input alongside autonomous navigation.

**Joystick Launch File:** `joystick.launch.py`  
- Enables the Joy Node to connect a joystick to the computer.
- Enables the Teleop Node to map joystick buttons for robot control.

## Joystick Setup Instructions

To configure the joystick for controlling the robot, check out this guide: https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/teleop/
   

## Usage 
### 1. **Start the Main Launch File**  
   ```ros2 launch robot_control_pkg diffbot.launch.py```
### 2. **Start the Joystick Launch File**  
   ```ros2 launch robot_control_pkg joystick.launch.py```

## Dependencies

Make sure the following dependencies are installed:

```bash
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install libserial-dev
sudo apt install joystick jstest-gtk evtest
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-twist-mux
