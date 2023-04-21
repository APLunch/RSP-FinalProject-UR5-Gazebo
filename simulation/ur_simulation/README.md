# UR Simulation
UR Simulation is an Ignition Gazebo simulation of a UR5 mainipulator robot with a Robotiq gripper in the real world. It is a fundamental necessity for the research project.

## Requirements
### Platform
- ROS2 Galactic
- Ignition Gazebo Fortress
- Dependency: ur_description

### Simulation world
- UR5 Robot mounted on a table
- Robotiq Gripper mounted to the UR5 robot end effector
- 2 Cameras (camera1, camera2) mounted in space, each have full view of the workspace
- 1 additional camera (camera3) mounted on robot wrist
### Sensors
- The camera parameters (fov, pose, etc.) should be configure-able by a config file
- The simulator package should publish sensor_msgs/Image to 3 topics ```/image1```, ```/image2``` and ```/image3```. Each corresponds to the footage of the cameras in the world.
 
## Details - ur_gazebo Package
```ur_gazebo``` contains the simulation world description files (sdf) and the associated launch files.

### The Gazebo World
The world sdf file is  ```ur_gazebo/world/ur_simulation_world.sdf```. 
The world consists of:
- A [table model](https://app.gazebosim.org/OpenRobotics/fuel/models/Table) from ignition fuel resource
- A UR5 robot model with a Robotiq 2f85 gripper
- Camera sensors as mentioned in the above section.

Robot is attached to the world rigidly via a fixed joint, example code:
```xml
<!-- Fix To World -->
<joint name="ur5_rg2_joint_world" type="fixed">
    <parent>world</parent>
    <child>base_link</child>
</joint>
```

The Robot is spawn via ```ros_ign_gazebo create``` executable which gets the robot description from ```/robot_description``` topic. 

Note that the environment variable for Gazebo must be set to ros2 share folder in order for it to locate ur5 mesh files in ```ur_description``` package.
Code:
```bash
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/galactic/share"
```

### Robot Description
The urdf file for the robot is generated from a modified ur xacro file  ```urdf/ur.urdf.xacro```. The file is originally from ur_description package but it is modified to include the gripper model and necessary sensors. Note that the robot bringup and the generation of the robot description from this file still depend on components of ```ur_description``` package.


## TODO: 
- Add gripper to the model
- Issue: Robotiq does not have ROS2 driver. It is not controlled in real life.
