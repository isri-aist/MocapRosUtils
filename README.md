# [MocapRosUtils](https://github.com/isri-aist/MocapRosUtils)
ROS-based utilities for motion capture

## Install

### Dependencies
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)

### Installation procedure
```bash
$ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Examples

### Dump URDF file from BVH hierarchy data
```bash
$ rosrun mocap_ros_utils GenerateUrdfFromBvh `rospack find mocap_ros_utils`/data/sample_walk.bvh /tmp/BvhRobot.urdf BvhRobot
# Visualize URDF model
$ roslaunch urdf_tutorial display.launch model:=/tmp/BvhRobot.urdf
```

### Visualize BVH motion with TF
```bash
$ roslaunch mocap_ros_utils visualize_bvh.launch
```
