# UR Simulation
UR Simulation is an Ignition Gazebo simulation of a UR5 mainipulator robot with a Robotiq gripper in the real world. It is a fundamental necessity for the research project.

## Requirements
### Platform
- ROS2 Galactic
- Ignition Gazebo Fortress
### Simulation world
- UR5 Robot mounted on a table
- Robotiq Gripper mounted to the UR5 robot end effector
- 2 Cameras (camera1, camera2) mounted in space, each have full view of the workspace
- 1 additional camera (camera3) mounted on robot wrist
### Input/Output
- The camera parameters (fov, pose, etc.) should be configure-able by a config file
- The simulator package should publish sensor_msgs/Image to 3 topics ```/image1```, ```/image2``` and ```/image3```. Each corresponds to the footage of the cameras in the world.
 
