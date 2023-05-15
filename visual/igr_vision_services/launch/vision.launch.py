from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths to the nodes and YAML file
    config_path = os.path.join(
        get_package_share_directory('igr_vision_services'), 'config', 'vision.yaml'
    )
    
    # Declare LaunchArgument for YAML file path
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=config_path,
        description='Full path to the YAML config file to be loaded'
    )

    # Define nodes
    feature_extraction_service = Node(
        package='igr_vision_services',
        executable='feature_extraction_service',
        output='screen'
    )
    
    stereo_vision_service = Node(
        package='igr_vision_services',
        executable='stereo_vision_service',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # Add all nodes to LaunchDescription
    ld = LaunchDescription()

    ld.add_action(declare_config_file_cmd)
    ld.add_action(feature_extraction_service)
    ld.add_action(stereo_vision_service)
    
    return ld