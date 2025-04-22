import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch diffbot
    diffbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("robot_control_pkg"),
                    "launch",
                    "diffbot.launch.py",
                )
            ]
        )
    )

    # Launch lidar
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("rplidar_ros"),
                    "launch",
                    "rplidar_a2m8_launch.py",
                )
            ]
        )
    )

    # Launch
    return LaunchDescription([
        diffbot,
        lidar
    ])