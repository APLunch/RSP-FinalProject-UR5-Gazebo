# UR Simulation
UR Simulation is an Ignition Gazebo simulation of a UR5 mainipulator robot with a Robotiq gripper in the real world. It is a fundamental necessity for the research project.

<img src="https://user-images.githubusercontent.com/60408626/233907555-662ae740-2485-483c-b73e-95e3158d82c3.png" width = 400> <img src="https://user-images.githubusercontent.com/60408626/233907640-ec122046-031d-4f26-8c2d-6b89b7cc95c0.png" width = 500>



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

## How-To
### Step 0: Install Dependencies
If not yet installed dependency packages, install all packages using ```rosdep```.
```bash
cd <workspace-path>
rosdep install --from-paths ./ -y --ignore-src
```
These command will download and install all dependency packages.

### Step 1: Build Packages
Before building the packages, we need to let the ```gz_ros2_control``` package acknowledge the version of Ignition Gazebo we are using (which is fortress) to ensure the plugin is built for with the correct version. 
```bash
export IGNITION_VERSION=fortress
```
Then we can proceed to build the packages.
```bash
colcon build 
```

### Step 2: Source Gazebo Resource path
```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/opt/ros/galactic/share
```
This step is necessry for gazebo to load resources from the ros2 directories.

Next we need to set the ```IGN_GAZEBO_SYSTEM_PLUGIN_PATH``` environment variable.

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:<path-to-workspace>/install/ign_ros2_control/lib
```
This step tells gazebo to look for system plugins inside ```install/ign_ros2_control/lib``` which is where the ```ign_ros2_control``` plugin library is located.

### Step 3: Source Built Packages
``` bash
source install/setup.bash
```

### Step 4: Launch Simulation
```bash
ros2 launch ur_gazebo ur_simulation.launch.py
```
A Gazebo smulation and a Rviz window with 2 camera windows should pop up. 

### Step 4.1: Start ROS2 Controller and Test (Optional)
Open a new terminal inside the workspace.
Start the ros2 controller 
```bash
ros2 control load_controller --set-state start ur5_controller
```
You should see a message for a successful controller initiation.

Then we publish a message to the position controller topic to let the controller move the robot to desired joint positions
```bash
ros2 topic pub /ur5_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.1, -1.0, -1.5, -1.8, 1.5, 0.0, 0.1, 0.1]"

```
The above example sets 6 joints of the ur5 to -1.0 rads, and each gripper finger joint to 0.3 rad.
The robot should then move to the commanded location.
 
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

The modified robot description includes gazebo joint state publisher plugin to accomodate visualization is Rviz.

In addition, a modified robotiq 2f 85 gripper is attached to the ```tool0``` frame of the robot. The description of the gripper is in ```ur_gazebo/urdf/robotiq_gripper.xacro```.
More specifically, the gripper's parallel structure is removed to acccomodate urdf which only support serial robotic structures. The fingers of the 2f gripper is no longer parallel but a pillar style instead.

<img src="https://github.com/APLunch/Intelligent-General-Robot/assets/60408626/93e5d3dd-104d-4e57-966a-8fc07a9a9e40" width = 400>

As for the controller for the gripper, the two fingers of the grippers are controller separately as joint 7 and joint 8.

### Cameras in Simulation World
<img src="https://user-images.githubusercontent.com/60408626/233908460-98a723db-387c-4975-946f-dd3a2273b281.png" width = 400>

Two cameras are mounted at a 90 degree angle looking at the robot. Each camera are condigured in a code block in 
```ur_gazebo/world/ur_simulation_world.sdf``` file. Parameters including pose, resolution, fov, etc. can be changed in the sdf file. Each camera has an ignition gazebo topic publishing their footage ``` /image1``` and ```/image2```. These topic are bridged to ROS2 topics with the same names.

### Launch File
The launch file ```launch/ur_simulation.launch.py``` is the launch file that bringup the simulation, spwan the robot, and setup the sensors.
It launches the following component sequentially:
1. robot_state_publisher (from robot description)
2. rviz node
3. ignition gazebo (from world sdf file)
4. spawn ur5 robot in simulation world
5. ign_ros_bridges (joint_states, image topics)

### ROS2 Controller
We use the ```gz_ros2_control``` package to manipulate the simulated ur5 robot. The package is downloaded from [ros-control repo](https://github.com/ros-controls/gz_ros2_control.git) and the source is saved locally.
Please see the README file in ```gz_ros2_control``` package for detailed tutorial on how to use it. As for now, the ```gz_ros2_control``` is configured and is ready to be used.

## TODO: 
- Issue: Robotiq does not have ROS2 driver. It is not controlled in real life.
- Add additional depth camera on camera3

