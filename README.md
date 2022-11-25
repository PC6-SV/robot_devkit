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
* Intel RealSense D400 Series, Intel RealSense T265

### System Requirements
* We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## 2. Installation Steps
### Packages
To clone a private repository, you need to generate a PAT(Personal Access Token) on your Github account, and add it to the command below.
```bash
git clone https://<pat>@github.com/PC6-SV/robot_devkit.git
cd robot_devkit
./demo/rdk_install.sh
```

### Sourcing
If this is the only means by which ROS2 is installed (which is most likely the case):
```bash
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
echo ". ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
```
To undo this, locate your system’s shell startup script and remove the appended commands.  
Otherwise, if multiple versions of ROS2 exist, run this command on every new shell you open to have access to the ROS 2 commands:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash
```
