# [MocapRosUtils](https://github.com/isri-aist/MocapRosUtils)
ROS-based utilities for motion capture

[![CI](https://github.com/isri-aist/MocapRosUtils/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/MocapRosUtils/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/MocapRosUtils/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/MocapRosUtils)](https://github.com/isri-aist/MocapRosUtils/blob/master/LICENSE)

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

https://github.com/isri-aist/MocapRosUtils/assets/6636600/c1c3b077-3f81-42dd-9d78-1073588b167a
