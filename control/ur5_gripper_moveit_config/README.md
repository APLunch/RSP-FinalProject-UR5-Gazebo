# ur5_gripper_moveit_config
![image](https://github.com/APLunch/Intelligent-General-Robot/assets/60408626/0978edde-63b4-48ab-8bd6-26c648fec36b)

## Introduction
This package is a [MoveIt!](https://moveit.picknik.ai/humble/index.html) configuration package which is necessary for the motion planning of the robot.

Traditionally in ROS1, this package is generated through the utilization of moveit_setup_assistant tool, which takes a urdf file as input and configures various MoveIt functionalities. However, it is important to note that MoveIt has not yet undergone complete migration to ROS2 Galactic distribution or other subsequent releases.

To address this limitation, a workaround method is employed to generate this package, as described in the guide [Build a MoveIt Package for ROS2](https://industrial-training-master.readthedocs.io/en/foxy/_source/session3/ros2/3-Build-a-MoveIt-Package.html).

## Getting Started
### Step 0: If not built, build the package

```bash
colcon build
source install/setup.bash
````
### Step 1: Launch the MoveIt moveroup Node and/or Rviz Node for Planning

To bring up regular MoveIt with planning GUI component

```bash
ros2 launch ur5_gripper_moveit_config ur5_gripper_planning_execution.launch.py 
```
TODO: Add another lunch file which launch without Rviz.

This should 

### Step 2: Start Planning
![image](https://github.com/APLunch/Intelligent-General-Robot/assets/60408626/039db04c-4dfc-4ef6-9368-c8d195088b36)

Please note that there is a distinction in time management between the movegroup and the simulation. The movegroup operates based on real ROS time, while the simulation employs its own simulation time. Regrettably, attempting to configure the moveit group to utilize simulation time has proven ineffective for reasons that remain unclear. Nevertheless, it has been observed that the trajectory controller functions appropriately when configured to utilize real ROS time. Consequently, it is imperative to execute the following command prior to executing the trajectory:

```bash
ros2 param set /manipulator_joint_trajectory_controller use_sim_time False
```

This prevents the trajectory controller to use simulation time because it is set to use simulation time by default.


