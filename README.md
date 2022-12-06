# Intel Robot DevKit

## 1. Introduction
Intel Robot DevKit is the tool to generate Robotics Software Development Kit (RDK) designed for autonomous devices, including the ROS2 core and capacibilities packages like perception, planning, control driver etc. It provides flexible build/runtime configurations to meet different autonomous requirement on top of diversity hardware choices, for example use different hareware engine CPU/GPU/VPU to accelerate AI related features.

After build, the RDK is installed on Ubuntu18.04 with below items on development machine for further development.
* ROS2 core
* ROS2 Object Analytics(OA) with RealSense(RS) camera
* ROS2 OpenVINO for people detection
* Gazebo 9 with Gazebo-ros2 simulator
* ROS2 navigation
* rviz2
* ROS2 tutorial for Intel components

### Hardware Requirements
* x86 CPU
* Intel RealSense D400 Series

### System Requirements
* We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## 2. Installation Steps
### Packages
Navigate to an empty directory which you intend to use as a workspace and run:
```bash
mkdir src
cd src
git clone https://github.com/PC6-SV/robot_devkit.git
cd robot_devkit
./rdk_install.sh
```

### Running
To run, go to workspace directory and execute:
```bash
source /opt/ros/dashing/setup.bash
. ./install/local_setup.bash
. ./src/robot_devkit/ros2_ws/install/local_setup.bash
. ./src/robot_devkit/rdk_ws/perception_ws/install/local_setup.bash
export ROS_DOMAIN_ID=120
ros2 launch realsense2_nav autonomy_launch.py
```
