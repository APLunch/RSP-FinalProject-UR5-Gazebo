from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import os
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ur_description'), "urdf", 'ur.urdf.xacro']),
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5",
            " ",
            "prefix:=",
            '""',
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ur_description'), "rviz", "view_robot.rviz"]
    )
    ops_path = PathJoinSubstitution([FindPackageShare('rtt_ur_trajectory'),'scripts', 'robot.ops'])
    deployer_launch = ExecuteProcess(
            cmd=['gnome-terminal','--', 'deployer', '-s', ops_path],output='screen',
            env=os.environ.copy()
        )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
        remappings=[('/joint_states', '/robot/joint')],
    )
    rqt_reconfigure_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        output="log",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    ld = LaunchDescription()
    #ld.add_action(robot_state_publisher_node)
    #ld.add_action(rviz_node)
    ld.add_action(deployer_launch)
    ld.add_action(rqt_reconfigure_node)
    return ld

