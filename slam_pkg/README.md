# SLAM Package

This package is designed for **Online Asynchronous SLAM**, using the `slam_toolbox` package of ROS2, where live data is processed in real-time. It always processes the **most recent scan**, ensuring efficiency even if some scans are skipped.

---

## Key Features

1. **Live Data Processing**  
   Unlike offline methods that use prerecorded data, this SLAM approach works with live sensor data, always prioritizing the most recent scan.

2. **`base_footprint` Frame in URDF**  
   - A `base_footprint` frame has been added to the robot's URDF.  
   - This frame represents the projection of the robot on the x-y plane, which is required for **2D SLAM**.

---

## Modifications to `slam_toolbox`

### `mapper_params_online_async.yaml`
- The `base_frame` parameter is set to:  
  ```base_frame: base_footprint_link```
### `online_async_launch.py`
- Created two files, one for simulation and one for the real robot (the only difference is that I set `use_sim_time` parameter to true and false accordingly).

## Usage 
### 1. **Start the SLAM Launch File**  
   ```ros2 launch slam_pkg slam_sim.launch.py```
   
   ```ros2 launch slam_pkg slam_real.launch.py```
   
The only difference is that the launch file for simulation sets `use_sim_time` to true and the other one sets it to false.

### Note
I also added the localization launch and parameters file but is not very good, it is better to use AMCL from NAV2 package.
## Dependencies

Make sure the following dependencies are installed:

```bash
sudo apt install ros-jazzy-slam-toolbox
