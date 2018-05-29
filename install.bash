#!/usr/bin/env bash

readonly caller_dir=$(pwd)

readonly build_dir=${caller_dir}/redbird_quadrotor_install
readonly catkin_root=${build_dir}/catkin_ws
readonly catkin_src_dir=${catkin_root}/src

readonly ros_distro_root=/opt/ros/kinetic


# install dependencies
sudo apt update
sudo apt install ros-kinetic-desktop-full
sudo apt install ros-kinetic-mavros*
sudo apt update

# get geography data
mkdir 
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# ensure ros env vars are set
. ${ros_distro_root}/setup.bash

# pre build catkin_tools
mkdir -p ${catkin_src_dir}
cd ${catkin_root}
catkin build
. devel/setup.bash

# clone source
git clone https://github.com/redbirdrobotics/redbird_quadrotor.git
cd redbird_quadrotor && git submodule update --init --recursive && cd -

mkdir -p ${catkin_src_dir}
cd ${catkin_src_dir}
git clone https://github.com/redbirdrobotics/redbird_quadrotor.git

# build source
cd ${catkin_root}
catkin build -j4
. devel/setup.bash

cd ${caller_dir}
