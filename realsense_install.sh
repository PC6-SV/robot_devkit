source /opt/ros/dashing/setup.bash
if [ ! -d "./ros2_ws/src/realsense-ros" ]; then
    mkdir -p ./ros2_ws/src
    cd ./ros2_ws/src/
    git clone https://github.com/PC6-SV/realsense-ros.git
    cd ..
    cd ..
fi
cd ros2_ws
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
rosdep install -i --from-path src/robot_devkit/realsense2-nav --rosdistro dashing --skip-keys="librealsense2 realsense_examples realsense2_camera" -y
colcon build --base-paths ./src/robot_devkit/realsense2-nav
