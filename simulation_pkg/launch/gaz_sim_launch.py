import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the urdf/xacro file path
    path_to_urdf = get_package_share_path('simulation_pkg')/'description'/'robot_urdf.xacro'
    
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]), value_type=str), 'use_sim_time': use_sim_time
        }]
    )

    #Use your custom Gazebo world (optional). Replace 'empty.sdf' with the word "world" (no "" needed)
    world = os.path.join(
        get_package_share_directory('simulation_pkg'), "worlds", "industrial-warehouse.sdf"
    )

    # Launch gazebo wolrd
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world]}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot1",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "1.4",
        ],
        output="screen",
    )

    # Bridge gazebo parameters with ROS
    bridge_params = os.path.join(
    get_package_share_directory('simulation_pkg'),
        'params',
        'ros_gz_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Run ros2 control spawner scripts
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_controller'],
    )

    twist_mux_params = os.path.join(get_package_share_directory('simulation_pkg'), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', 'diff_controller/cmd_vel')]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        diff_controller_spawner,
        start_gazebo_ros_bridge_cmd,
        joint_state_broadcaster_spawner,
        twist_mux
    ])