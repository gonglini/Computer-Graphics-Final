#!/bin/bash
# One-time setup inside WSL2 Ubuntu 20.04 (run as root):
#   wsl -d Ubuntu-20.04 -u root -- bash /mnt/c/.../tools/slam/setup_fastlio.sh
# ROS Noetic + Livox SDK / livox_ros_driver (FAST-LIO needs its message definitions) + FAST_LIO.
set -e
export DEBIAN_FRONTEND=noninteractive
HERE="$(cd "$(dirname "$0")" && pwd)"
WS=/opt/fastlio_ws

apt-get update
apt-get install -y curl gnupg2 lsb-release git build-essential cmake
if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
  echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
  apt-get update
fi
apt-get install -y ros-noetic-ros-base ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf \
                   ros-noetic-eigen-conversions ros-noetic-rosbag libpcl-dev libeigen3-dev python3-catkin-tools

# Livox SDK (library used by livox_ros_driver)
if [ ! -d /opt/Livox-SDK ]; then
  git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git /opt/Livox-SDK
  cd /opt/Livox-SDK/build && cmake .. && make -j"$(nproc)" && make install
fi

source /opt/ros/noetic/setup.bash
mkdir -p "$WS/src"
cd "$WS/src"
[ -d livox_ros_driver ] || git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git
if [ ! -d FAST_LIO ]; then
  git clone https://github.com/hku-mars/FAST_LIO.git
  cd FAST_LIO && git submodule update --init && cd ..
fi
# our sensor configuration + launch file
cp "$HERE/sim.yaml" FAST_LIO/config/sim.yaml
cp "$HERE/mapping_sim.launch" FAST_LIO/launch/mapping_sim.launch
cd "$WS"
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"
echo "FAST_LIO built in $WS"
