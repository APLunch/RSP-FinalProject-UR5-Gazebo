# Launch file that bring up sub-launch files for simulation and moveit
# Simulation launch file is located in package: ur_gazebo/launch/ur_simulation.launch.py
# Moveit launch file is located in package: ur5_gripper_moveit_config/launch/ur5_gripper_planning_execution.launch.py

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Launch simulation
    simulation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package='ur_gazebo'), 'launch', 'ur_simulation.launch.py'])
        )
    )

    # Launch moveit
    moveit_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package='ur5_gripper_moveit_config'), 'launch', 'ur5_gripper_planning_execution.launch.py'])
        )
    )


    #Ensure set_param_command is executed after simulation_launch_file and moveit_launch_file

    ld = LaunchDescription()
    ld.add_action(simulation_launch_file)
    ld.add_action(moveit_launch_file)


    return ld

