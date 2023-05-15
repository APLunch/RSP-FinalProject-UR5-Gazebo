from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from ament_index_python.packages import get_package_share_path
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import os

def generate_launch_description():
    # Launch ur5 description
    # UR5 robot description view bring up from the package launch file
    ur5_launch_file = os.path.join(FindPackageShare("ur_description").find("ur_description"), 'launch', 'view_ur.launch.py')
    # UR5 launch
    ur5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur5_launch_file),
        launch_arguments={'ur_type': 'ur5'}.items(),
    )
    # Get UR5 robot description
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur5",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_gazebo",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " ",
        "safety_limits:=",
        safety_limits,
        " ",
        "safety_pos_margin:=",
        safety_pos_margin,
        " ",
        "safety_k_position:=",
        safety_k_position,
        " ",
        "name:=",
        "ur",
        " ",
        "ur_type:=",
        ur_type,
        " ",
        "prefix:=",
        prefix,
    ]
    )
    robot_description = {"robot_description": robot_description_content}
    # UR5 Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/joint_states", "/world/ur_simulation_world/model/ur5/joint_state")],
    )
    # RViz2 Node with UR5
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Start Gezbo world
    gazebo = ExecuteProcess( 
        cmd=['ign','gazebo',FindPackageShare(package='ur_gazebo').find("ur_gazebo")+ "/world/ur_simulation_world.sdf"],
        output='screen'
    )

    #Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_ign_gazebo', 
        executable='create',
        arguments=['-world', 'ur_simulation_world', '-topic', '/robot_description', '-name', 'ur5' ,' -x', '0', '-y', '0', '-z', '1.05'],
    )

    #Create ign ros bridge for joint state publisher
    ign_ros_bridge_joint_states = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/world/ur_simulation_world/model/ur5/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model'],
        output='screen'
    )

    # Create ign ros bridge for /image1 topic
    ign_ros_bridge_image1 = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/image1@sensor_msgs/msg/Image@ignition.msgs.Image'],
        output='screen'
    )

    # Create ign ros bridge for /image2 topic
    ign_ros_bridge_image2 = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge', 
        arguments=['/image2@sensor_msgs/msg/Image@ignition.msgs.Image'],
        output='screen'
    )

    # Create ign ros bridge for /rgbd_camera/image and /rgbd_camera/depth_image topics
    ign_ros_bridge_image3 = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge', 
        arguments=['/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image'],
        output='screen'
    )

    return LaunchDescription([
        #ur5_launch, #This launches the original ur5 description
        *declared_arguments,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_robot,
        ign_ros_bridge_joint_states,
        ign_ros_bridge_image1,
        ign_ros_bridge_image2,
        ign_ros_bridge_image3
    ])