# RSP Final Project - UR5 Gazebo Simulation for Pick and Place Tasks

<img src="https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/9e6da746-9590-4483-966e-938d9ceffc53" width = 400>
<img src="https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/034ace53-f08f-4f01-b5d7-0a68edf04d9e" width = 400>
<img src="https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/ff16d9db-b5a6-46a6-a2a6-d5c76fe78e19" width = 400>
<img src="https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/397fa22a-b293-4529-bee8-c2b16a7e4155" width = 400>

## Overview
Welcome to my GitHub project for the Robot System Programming course. This project showcases a comprehensive set of key features designed to enhance the functionality and control of a UR5 robotic arm in a simulated environment using ROS2 Galactic.

The project's main accomplishments include the construction of a UR5 Gazebo simulation that is fully compatible with ROS2 Galactic. This simulation enables seamless control of the simulated UR5 robot through ROS2 Controllers, allowing for precise manipulation and movement.

Within the simulation world, a realistic environment has been created, featuring a table to which the robot arm is attached. Additionally, various objects are scattered on the table, presenting challenging pick-and-place scenarios for the robot.

To provide a visual perspective, cameras have been strategically placed within the simulation world, streaming images via ROS. Furthermore, the robot itself is equipped with a camera and a depth sensor, both streaming images and depth data via ROS.

To enhance the robot's manipulation capabilities, a 2-finger gripper has been seamlessly integrated into the URDF (Unified Robot Description Format) of the robot's end effector.

Moreover, the project incorporates MoveIt motion planning, enabling efficient path planning and collision avoidance for the robot. As a convenient feature, a table-collision object is automatically added upon the project's initialization.

To simplify the process of controlling the robot, a dedicated "Move-Robot" package has been developed. This package integrates move-plan-execute functionalities into a single message, streamlining the robot's movements and providing an intuitive interface for users.

Overall, this GitHub project demonstrates a robust and versatile implementation of a UR5 robot system, combining powerful features such as ROS2 compatibility, simulated environment with objects, camera and depth sensor integration, motion planning with MoveIt, and a user-friendly control package.

## Significance

### Simulation Capabilities
The project serves as a valuable resource for students and researchers, offering a simulated environment that allows them to test and develop robotic applications without the need for physical robots. This accessibility fosters learning and experimentation in the field of robot system programming, making it an essential asset for future students and researchers.

### Enhanced Perception
The inclusion of sensors within the simulation, particularly the cameras, enables visual perception of the robot's workspace. This broadens the scope of potential applications, particularly those involving computer vision in robotics. The ability to perceive and analyze the environment visually opens up possibilities for advanced algorithms and applications in areas such as object recognition, tracking, and manipulation.

### Debugging and Integration
The project addresses the challenges posed by the incomplete migration of critical components, such as the Gazebo ROS2 control package and MoveIt, to ROS2. Through various debugging processes, the project successfully clears a path for integrating ROS controllers with Gazebo Ignition and integrates MoveIt with Gazebo. This paves the way for future developers to navigate and overcome similar obstacles, contributing to the advancement and integration of these important tools in ROS2.

### Smart Pick-n-Place Development
As a culmination of its features, this project provides a playground for future advancements in smart pick-n-place tasks. The combination of simulated robotics, perception sensors, integrated motion planning, and the incorporation of a gripper enables the exploration and development of innovative solutions in the field of autonomous pick-and-place operations. This serves as a foundation for future research and development in automation and robotics.

## Getting Started
### Step 0: Clone Repository
```bash
cd <your workspace>
```
```bash
git clone https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo.git
```
### Step 1: Install Dependencies
Install all UR-related Packages
```bash
sudo apt-get install ros-galactic-ur-*
```
Install all Moveit Packages
```bash
sudo apt-get install ros-galactic-moveit*
```
Install dependencies using rosdep
```bash
cd <your workspace>
```
```bash
rosdep install --from-paths ./ -y --ignore-src
```

### Step 2: Build Packages
Before building the packages, we need to let the ```gz_ros2_control``` package acknowledge the version of Ignition Gazebo we are using (which is fortress) to ensure the plugin is built for with the correct version. 
```bash
export IGNITION_VERSION=fortress
```
Then we can proceed to build the simulation-control related packages.
```bash
colcon build --packages-select ign_ros2_control ur5_gripper_moveit_config ur_gazebo move_robot bringup
```

### Step 3: Source Gazebo Resource path
```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/opt/ros/galactic/share
```
This step is necessry for gazebo to load resources from the ros2 directories.

Next we need to set the ```IGN_GAZEBO_SYSTEM_PLUGIN_PATH``` environment variable.

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:<path-to-workspace>/install/ign_ros2_control/lib
```
This step tells gazebo to look for system plugins inside ```install/ign_ros2_control/lib``` which is where the ```ign_ros2_control``` plugin library is located.

### Step 4: Source Built Packages and Launch Project
``` bash
source install/setup.bash
ros2 launch bringup simulation_moveit_bringup.launch.py 
```
This should bringup the gazebo world, a Rviz window for cameras, and another Rviz window for MoveIt Planning.
Click the 'play' botton in Gazebo to start the simulation world.

Note that the table obstacle object is already spawned in the planning scene.

### Step 5: One Last Step , and Start Planning!
Please note that there is a distinction in time management between the movegroup and the simulation. The movegroup operates based on real ROS time, while the simulation employs its own simulation time. Regrettably, attempting to configure the moveit group to utilize simulation time has proven ineffective for reasons that remain unclear. Nevertheless, it has been observed that the trajectory controller functions appropriately when configured to utilize real ROS time. Consequently, it is imperative to execute the following command prior to executing the trajectory:
```bash
ros2 param set /manipulator_joint_trajectory_controller use_sim_time False
```
![image](https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/b8f0d5db-8a81-4dcf-bfc6-1810d823aa63)

### Step 6 (Optional): Use command line to move the robot
The package ```move_robot``` provides an interface to control the robot more easily by only publishing one ```PoseStamped``` message to its topic ```/move_robot/move_robot_fnf```

Thes ```move_robot``` node is started together with ```MoveIt``` in step 4, we can play with in by publishing a command to the topic:
```bash
ros2 topic pub --once /move_robot/move_robot_fnf geometry_msgs/msg/PoseStamped "{
  header: {
    frame_id: 'base_link'
  },
  pose: {
    position: { 
      x: -0.45368513315032716, 
      y: 0.06996090974584167, 
      z: 0.23476112703003504 },
    orientation: { 
      x: 0.8624911053036711, 
      y: 0.486167430188559, 
      z: -0.14052325972808613, 
      w: -0.0018805773840037013
    }
  }
}"

```

This should move the robot to look at the objects placed on the table.

![image](https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/a5316cae-3534-4458-b6a6-5ae7f19d8722)

Note that this is a *fire-and-forget* function and it only proceed with the attempt, and the result of the planning or the execution is not guaranteed.

### Step 7 (Optional) Play with other Add-on features
The above steps concluded the setup for the simulation project. On the other hand, we have been working on some features that involves AI and Computer Vision.
#### GPT-Integrated Control
The GPT-Integrated Control allows the user to chat with chatGPT and let it control robots to perform pick-and-place tasks like shown in the demo video below.
As the development is still on-going, the program make sufficient assumptions on the object's poses.

If you would like to try this, please switch branch to ```gpt_rtt_demo``` and follow [these steps](https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/blob/gpt_rtt_demo/simulation/ur_simulation/README.md) for simulation setup and  [these steps](https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/files/11519231/communication.pptx) for gpt node to build and run the GPT-Integrated Control.**Note: It seems that the GPT Key has to be private and it will be invalid once pushed to the repo along with the config file, please contact me if you would like to try this feature so I can distribute the key to you off-line**

The demo video:

https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/64ce0477-0d6e-4fb6-b0e6-7fc209a5f521

#### Object Detection and Location using SAM
This feature utilizes the cameras and the depth sensor on the robot to provide perception of the robot workspace. The idea is that we use [SAM](https://github.com/facebookresearch/segment-anything) (Segment Anything Model) to detect objects in the image, and locate the object's location relative to the robot frame using depth camera. 

At current stage, it can detect and publish object positions placed on the table.

<img src="https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/d5afdd28-7def-4ecd-8207-fbff6e3c12cb" width=400>
<img src="https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/assets/60408626/cc519fe8-b44d-4d46-88ef-59aaeff7577d" width=400>

If you want to explore it, follow [these steps](https://github.com/APLunch/RSP-FinalProject-UR5-Gazebo/tree/master/visual) to build and run the igr vision service.

## Project Significance

### Simulation Capabilities
The project serves as a valuable resource for students and researchers, offering a simulated environment that allows them to test and develop robotic applications without the need for physical robots. This accessibility fosters learning and experimentation in the field of robot system programming, making it an essential asset for future students and researchers.

### Enhanced Perception
The inclusion of sensors within the simulation, particularly the cameras, enables visual perception of the robot's workspace. This broadens the scope of potential applications, particularly those involving computer vision in robotics. The ability to perceive and analyze the environment visually opens up possibilities for advanced algorithms and applications in areas such as object recognition, tracking, and manipulation.

### Debugging and Integration
The project addresses the challenges posed by the incomplete migration of critical components, such as the Gazebo ROS2 control package and MoveIt, to ROS2. Through various debugging processes, the project successfully clears a path for integrating ROS controllers with Gazebo Ignition and integrates MoveIt with Gazebo. This paves the way for future developers to navigate and overcome similar obstacles, contributing to the advancement and integration of these important tools in ROS2.

### Smart Pick-n-Place Development
As a culmination of its features, this project provides a playground for future advancements in smart pick-n-place tasks. The combination of simulated robotics, perception sensors, integrated motion planning, and the incorporation of a gripper enables the exploration and development of innovative solutions in the field of autonomous pick-and-place operations. This serves as a foundation for future research and development in automation and robotics.


## Resources
[How to setup moveit for ros2 foxy](https://industrial-training-master.readthedocs.io/en/foxy/_source/session3/ros2/3-Build-a-MoveIt-Package.html) (Works for galactic as well).
