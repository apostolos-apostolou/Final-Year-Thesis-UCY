from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # Get URDF via xacro
    
    # Get the urdf/xacro file path
    path_to_urdf = get_package_share_path('robot_control_pkg')/'urdf'/'diffbot.urdf.xacro'

    robot_description = {'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]), value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_control_pkg"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    
    twist_mux_params = PathJoinSubstitution(
        [
            FindPackageShare("robot_control_pkg"),
            "config",
            "twist_mux.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--param-file", robot_controllers],
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', 'diff_controller/cmd_vel')]
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        twist_mux,
    ]

    return LaunchDescription(nodes)
