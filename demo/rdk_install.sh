#!/bin/bash

set -e

./rdk.sh config --default
./rdk.sh install-deps
./rdk.sh sync-src
./rdk.sh build --cmake-args -DCMAKE_BUILD_TYPE=Release
./rdk.sh install

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/ros2_ws
sudo apt-get install python3-rosdep -y
sudo rosdep init --include-eol-distros
sudo rosdep update --include-eol-distros
sudo rosdep fix-permissions
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
colcon build

# Generate tarball and install to remote device
# ./tools/generate_tarball.sh
# ./rdk.sh deploy --user rdktest --host 10.239.89.4 --file rdk_ws/release/rdk_release_201908051110.tar.gz --proxy http://<ip>:<port>

