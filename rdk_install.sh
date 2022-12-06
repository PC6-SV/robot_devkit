#!/bin/bash

set -e

./rdk.sh config --default
./rdk.sh install-deps
./rdk.sh sync-src
./rdk.sh build --cmake-args -DCMAKE_BUILD_TYPE=Release
./rdk.sh install

mkdir -p ./ros2_ws/src
cd ./ros2_ws/src/
git clone https://github.com/PC6-SV/realsense-ros.git
cd ..
sudo apt-get install python3-rosdep -y
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init --include-eol-distros
fi
sudo rosdep fix-permissions
rosdep update --include-eol-distros
rosdep install -i --from-path src --rosdistro dashing --skip-keys=librealsense2 -y
colcon build
cd ..
cd ..
cd ..
source /opt/ros/dashing/setup.bash
rosdep install -i --from-path src/robot_devkit/realsense2-nav --rosdistro dashing --skip-keys=librealsense2 -y
colcon build --base-paths ./src/robot_devkit/realsense2-nav

# Generate tarball and install to remote device
# ./tools/generate_tarball.sh
# ./rdk.sh deploy --user rdktest --host 10.239.89.4 --file rdk_ws/release/rdk_release_201908051110.tar.gz --proxy http://<ip>:<port>

