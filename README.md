# RSP Final Project - UR5 Gazebo Simulation for Pick and Place Tasks

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

## Requirements
The following requirements should be met for this project:
- Use ROS2 Galactic for this project. All inter-package communications must be via ROS2.
- Implement a high-level control package that comntrols the robot to perform simple tasks such as go to a certain pose, open/close gripper, etc. Motion planning and trajectory generation should be considered.
- The AI is trained or restricted to only perform tasks asked by user. That is, the AI is only allow to decide among the high-level commands that users have given it.
- The visual information of the robot workspace is fed to the AI at a moderate rate to ensure context awareness and decision reasoning.
- The AI is trained or configured such it understands user natrual language and outout commands to the robots.
- The robot is able to complete tasks (i.e. pick and place for manipulator robot) based on AI command.

## Structure
![image](https://user-images.githubusercontent.com/60408626/231640079-d7932104-3566-415c-9b9a-4c6cf317b02a.png)

### Components
The components (folders) of this project are listed as follow:
- **visual**: This folder contains packages that involve environmental sensing using cameras, including object detection, segmentation, and tracking. For more details, please refer to the README file inside the visual folder.
- **control**: This folder contains control packages that involve moving and manipulating robots at a lower level. These packages take high-level control commands selected by the AI as input and drive the robot. Examples of packages in this folder include UR5 controller, mobile robot controller, trajectory generators, and motion planning modules. Please see the README file inside the control folder for more information.
- **comminication**: This folder contains packages related to AI reasoning models, human input interfaces, and higher-level robot command output ports.
- **simulation**: This folder contains simulations for robots to be controlled. It's essential to test the viability of the robots in the simulator before deploying them in the real world. For UR5 simulation, see [this page](https://github.com/APLunch/Intelligent-General-Robot/blob/master/simulation/ur_simulation/README.md).

## How-To
1. Install ROS2 galactic following [this link](https://docs.ros.org/en/galactic/Installation.html).

## Resources
[How to setup moveit for ros2 foxy](https://industrial-training-master.readthedocs.io/en/foxy/_source/session3/ros2/3-Build-a-MoveIt-Package.html) (Works for galactic as well).
