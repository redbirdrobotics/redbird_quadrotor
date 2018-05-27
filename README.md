# redbird_quadrotor
Autonomous flight control for quadrotor drones.

## Build & run instructions
### Assumptions:
+ `mavros` and `mavlink` can be found by `rospack` (for `roslaunch`). You can either:
  + [build from source](https://github.com/mavlink/mavros/blob/master/mavros/README.md#source-installation) and source the resulting `<catkin_workspace>/devel/setup.bash` after build
  + [install prebuilt binaries](https://github.com/mavlink/mavros/blob/master/mavros/README.md#binary-installation-deb)

### Build the Gazebo simulation and Mavros node
```sh
git clone https://github.com/redbirdrobotics/px4-firmware.git && \
cd px4-firmware && \
make posix_sitl_default gazebo # Ctrl+C after gazebo starts
```

### Start the Gazebo simulation and Mavros node
```sh
cd px4-firmware && \
source setup_px4_sitl_gazebo.bash && \
roslaunch px4 mavros_posix_sitl.launch
```

or run our own simulated world:

```sh
cd redbird_quadrotor # from this repository's root
cd <path/to/px4-firmware> && \ # insert path here
. setup_px4_sitl_gazebo.bash && \
cd - && \
. devel/setup.bash && \
roslaunch redbird_quadrotor_controller gazebosim.launch
```


### Start the flight control node
```sh
rosrun redbird_quadrotor_controller redbird_quadrotor_controller_node
```


## Glossary
+ **SITL**: Software-in-the-loop
