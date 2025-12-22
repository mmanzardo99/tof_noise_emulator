# tof_noise_emulator

## Overview

`tof_noise_emulator` is a ROS1 package that implements a **PointCloud2 noise emulator**, primarily intended for **Time-of-Flight (ToF)** or short-range LiDAR sensors.

The main node:
- subscribes to **one or more input point clouds**;
- **removes points** inside a configurable region of interest (CropBox) defined in a target TF frame;
- **adds radial Gaussian noise** to the remaining points, proportional to their distance from the sensor;
- publishes a new "noisy" point cloud for each input topic.

---

## Features

- Support for **multiple input topics**
- **Range-dependent Gaussian noise** (σ = noise_scale · range)
- Spatial filtering using **PCL**


---

## Package Structure

```text
tof_noise_emulator/
├── src/
│   └── pointcloud_noise_node.cpp   # Main C++ node
├── launch/
│   └── tof_noise.launch             # Launch file
├── config/
│   └── noise_params.yaml            # Node parameters
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## ROS Node

### `pointcloud_noise_cpp`

C++ node that:
1. Receives PointCloud2 messages from one or more topics
2. Transforms the cloud into a target TF frame
3. Removes points inside a box centered at the target frame origin
4. Applies radial noise to the remaining points
5. Transforms the cloud back to the original frame and publishes it

---

## Topics

### Subscribed Topics

- All topics listed in `input_topics`
- Type: `sensor_msgs/PointCloud2`

### Published Topics

- `<input_topic>_noisy`
- Type: `sensor_msgs/PointCloud2`

Example:

```text
/tof1/points  → /tof1/points_noisy
/tof2/points  → /tof2/points_noisy
```

---

## Parameters

All parameters are loaded in the node private namespace.

### Main Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `input_topics` | list<string> | — | List of input PointCloud2 topics |
| `noise_scale` | double | 0.01 | Noise scale factor (σ = noise_scale · range) |
| `tf_target_frame` | string | `face_shell_front` | TF frame used for CropBox filtering |

### CropBox Parameters

The box is **centered at the origin of the target frame**.

| Name | Type | Description |
|------|------|-------------|
| `box_size_x` | double | Half-extent along X |
| `box_size_y` | double | Half-extent along Y |
| `box_size_z` | double | Half-extent along Z |

Points **inside the box are removed**.

---

## Configuration File

### `config/noise_params.yaml`

```yaml
pointcloud_noise_cpp:
  input_topics:
    - /tof1/points
    - /tof2/points
    - /tof3/points
    - /tof4/points
    - /tof5/points
    - /tof6/points
    - /tof7/points
    - /tof8/points
  noise_scale: 0.01
  box_size_x: 0.2
  box_size_y: 0.4
  box_size_z: 1.5
  tf_target_frame: "face_shell_front"
```

---

## Launch File

### `launch/tof_noise.launch`

```xml
<launch>
  <rosparam file="$(find tof_noise_emulator)/config/noise_params.yaml" command="load"/>

  <node pkg="tof_noise_emulator"
        type="tof_noise_node"
        name="pointcloud_noise_cpp"
        output="screen" />
</launch>
```

---

## Build Instructions

From your catkin workspace:

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

Main dependencies:
- roscpp
- sensor_msgs
- pcl_ros
- pcl_conversions
- tf2_ros
- Eigen

---

## Running the Node

```bash
roslaunch tof_noise_emulator tof_noise.launch
```

Verify published topics:

```bash
rostopic list | grep noisy
```

Visualization (RViz):
- Add a **PointCloud2** display
- Select `<input_topic>_noisy`

---

## Technical Notes

- Noise is **radial**, preserving the original point direction
- TF lookup is performed **at the message timestamp** (`msg->header.stamp`)
- If the required TF is unavailable, the cloud is dropped
- CropBox filtering is applied in the `tf_target_frame` coordinate system

---
