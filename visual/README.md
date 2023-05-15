# IGR Visions 

Classical and deep learning based computer vision packages for vision-guided robot manipulation. An RGBD camera is mounted on the end effector of the UR5. We used Segment Anything Model (SAM) with Grounding DINO to detect and segment object given text prompt in the RGB image view. Useful features are extracted from the detection bounding boxes and segmentation masks. The depth image is then utilized to find the 3D position of the object. Either a simple feedforward motion or a visual servoing control can be applied for potential manipulation tasks followed.


## Installation

### Install dependencies
```bash
pip install opencv-python pycocotools matplotlib onnxruntime onnx Pillow ipykernel
```

### Install torch
- if cuda
```bash
pip3 install torch torchvision torchaudio
```
- if cpu
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

### Install grounded-sam

```bash
pip install git+https://github.com/IDEA-Research/GroundingDINO.git
pip install git+https://github.com/facebookresearch/segment-anything.git
```

### Download models

_direct to the path_
```bash
cd ./visual/igr_vision_services/models
```
_wget model checkpoints_
```bash
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
wget https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
```

## Quick Start

### Build and source workspace

Under your ros2 workspace home directory, run

```bash
colcon build
source install/setup.bash
```

### Start vision services

The following command starts _feature extraction service_ and _Stereo vision service_. 

```bash
ros2 launch igr_vision_services vision.launch.py
```

It will take some time to load and warmup the neural network models, especially when GPU is not available. ROS messages will show up in the command window when services are ready.

### Run test clients

**Feature Extraction**

The _feature extraction test cleint_ takes a text prompt and an ROS2 topic, prints center positions and principal axes for each possible object in the image view described by the text prompt, and draws the segmentation mask and feature points and lines with that highest score. An example run is as follows

```bash
ros2 run igr_vision_services feature_extraction_test_client --ros-args -p text_prompt:="red ball" -p image_topic:="/image1"
```
That will extract features with the text prompt "red ball" in the view given by "/image1" topic.


**Stereo Vision**

The _stereo vision test client_ takes a text prompt, publishs the transforms of all possible objects match that text prompt into the tf2_ros tree. The resulting transforms can be viewed in Rviz. To run,

```bash
ros2 run igr_vision_services stereo_vision_test_client --ros-args -p text_prompt:="brown table"
```

Transforms corresponds to the text prompt "brown table" will be published.


## Developments

### How it works


### Configuration files

To configure the stereo vision service, navigate to the "config" directory under package "igr_vision_services"

```bash
cd ./visual/igr_vision_services/config
```
Open the file _vision.yaml_, the following parameters can be modified

```yaml
stereo_vision_service:
  ros__parameters:
    RGB_image_topic:
      "/rgbd_camera/image"
    depth_image_topic:
      "/rgbd_camera/depth_image"
    camera_optical_frame:
      "RGBD_optical_link"
    reference_frame:
      "world"
    simulation_on:
      True
    camera_intrinsic:
      cx: 480.0
      cy: 360.0
      fx: 831.57 # fx = width / (2 * tan(horizontal_fov / 2))
      fy: 831.57
```
Note that _camera_optical_frame_ is the optical center frame of the RGBD camera, _reference_frame_ is at the bottom of the transform tree. Camera intrinsics can be evaluated by camera calibration, where (cx, cy) is the principal point, fx and fy is the focal length (will be equal if square pixels are assumed).
