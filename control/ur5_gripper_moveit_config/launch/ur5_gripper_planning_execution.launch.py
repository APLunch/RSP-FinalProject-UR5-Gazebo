import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    xacro_file = get_package_file('ur_gazebo', 'urdf/ur.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    srdf_file = get_package_file('ur5_gripper_moveit_config', 'config/ur.srdf')
    kinematics_file = get_package_file('ur5_gripper_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('ur5_gripper_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_file = get_package_file('ur5_gripper_moveit_config', 'config/controllers.yaml')
    ros_controllers_file = get_package_file('ur5_gripper_moveit_config', 'config/ros_controllers.yaml')

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_execution_duration_scaling': 9.0,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.00,
    }
    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,

    }

    # MoveIt node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
                'ompl': ompl_config,
                'planning_pipelines': ['ompl'],
                #'use_sim_time': True,
            },
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
    )
    # TF information
    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    # Visualization (parameters needed for MoveIt display plugin)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur5_gripper_moveit_config"), "config", "rviz_config.rviz"]
    )
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
            }
        ],
        arguments=["-d", rviz_config_file],
    )
    # Controller manager for realtime interactions
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters= [
            {'robot_description': robot_description},
            ros_controllers_file
        ],
        output="screen",
    )
    # Startup up ROS2 controllers (will exit immediately)
    controller_names = ['manipulator_joint_trajectory_controller']
    spawn_controllers = [
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[controller],
            output="screen")
        for controller in controller_names
    ]

    # Setup scene in moveit
    #ros2 run moveit_ros_planning moveit_publish_scene_from_text ${scene_file} --scene --ros-args -p robot_description:="${urdf}" -p robot_description_semantic:="${srdf}"
    setup_scene = Node(
        package='moveit_ros_planning',
        executable='moveit_publish_scene_from_text',
        output='screen',
        arguments=['--scene',get_package_file('ur5_gripper_moveit_config', 'config/table.scene')],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
        ]
    )
    
    return LaunchDescription([
        move_group_node,
        #robot_state_publisher,
        #ros2_control_node,
        rviz,
        setup_scene,
        ] + spawn_controllers
    )