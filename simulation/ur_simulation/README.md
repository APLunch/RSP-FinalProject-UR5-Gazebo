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
### Sensors
- The camera parameters (fov, pose, etc.) should be configure-able by a config file
- The simulator package should publish sensor_msgs/Image to 3 topics ```/image1```, ```/image2``` and ```/image3```. Each corresponds to the footage of the cameras in the world.
 
## Details
### ur_gazebo
- ```ur_gazebo``` contains the simulation world description files (sdf) and the associated launch files.
- The world sdf file is  ```ur_gazebo/world/ur_simulation_world.sdf```
- Included a [table model](https://app.gazebosim.org/OpenRobotics/fuel/models/Table) from ignition fuel resource.
- Use a fixed joint to the world to mount the robot/object, example code:
```
<!-- Fix To World -->
<joint name="ur5_rg2_joint_world" type="fixed">
    <parent>world</parent>
    <child>base_link</child>
</joint>
```
TODO: Should I hard code robot model here or should I spwan it?
