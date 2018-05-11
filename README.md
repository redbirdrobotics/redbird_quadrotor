# gazebo-sim
Gazebo simulation for IARC Mission 7a

## Build & run instructions
### Assumptions:
+ `mavros` and `mavlink` can be found by `rospack` (for `roslaunch`). You can either:
  + [build from source](https://github.com/mavlink/mavros/blob/master/mavros/README.md#source-installation) and source the resulting `<catkin_workspace>/devel/setup.bash` after build
  + [install prebuilt binaries](https://github.com/mavlink/mavros/blob/master/mavros/README.md#binary-installation-deb)

```sh
git clone https://github.com/redbirdrobotics/px4-firmware.git && \
cd px4-firmware && \
make posix_sitl_default

# After build, gazebo will automatically run. Ctrl+C the terminal and wait for shutdown
# Note: make will say the build failed, but this has not caused issues this far. Probably ok to ignore.

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo && \
roslaunch px4 mavros_posix_sitl.launch # start the software-in-the-loop simulation
```

## Glossary
+ **SITL**: Software-in-the-loop
