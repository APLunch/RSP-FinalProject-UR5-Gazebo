# Intelligent General Robot

## Introduction
Welcome to the Intelligent General Robot project, an innovative research initiative aimed at integrating state-of-the-art Large Language Models (LLMs) and multi-modal AI systems, such as GPT-4, into the control loop of robotic systems. Our goal is to create versatile manipulators and mobile robots capable of understanding and executing tasks described using natural human language. This project encompasses various cutting-edge technologies, including computer vision for object detection, segmentation, and tracking, as well as simulations and ROS2 for seamless integration and communication within the robotic ecosystem. By harnessing the power of LLMs, our Intelligent General Robot project aspires to bridge the gap between human language and robotic capabilities, thereby revolutionizing the way we interact with and utilize robotics in our everyday lives.

## Notes:
- **All developers and reseachers, please document your work and structure moderately in each folder or package README file that you are working on For example, if you are developing the an object detection package in the visual folder, document the high level structure in ```visual/README.md```, and also include all details in ```visual/object_detection/README.md```**

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
- **simulation**: This folder contains simulations for robots to be controlled. It's essential to test the viability of the robots in the simulator before deploying them in the real world.

## How-To
1. Install ROS2 galactic following [this link](https://docs.ros.org/en/galactic/Installation.html).
