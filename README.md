# Marvin

This repository includes the code for our 2022-2023 robot, codename Marvin.

## Contents

- [Clone the Package](#clone-the-package)
- [Dependencies](#dependencies)
- [Set Up Cartographer](#set-up-cartographer)
- [Build the Workspace](#build-the-workspace)
- [Launch the Simulation](#launch-the-simulation)
- [Launch the Robot](#launch-the-robot)
- [Control the Robot](#control-the-robot)

## Clone the Package

Make sure you are in the `src` folder of an existing catkin workspace.

```bash
git clone https://github.com/umigv/marvin.git
```

## Dependencies
  
- ROS: `sudo apt install ros-noetic-desktop-full`
- Xacro: `sudo apt install ros-noetic-xacro`
- Gazebo (for simulation): `sudo apt install ros-noetic-gazebo-ros`
- Rviz: `sudo apt-get install ros-noetic-rviz`
- Required ROS Packages (clone each into `src` folder of catkin workspace for now, will set up proper dependencies later—there may be some missing dependencies which you will see when you run `catkin build`, just look up how to install them if so)
    - `controls_stack`: [https://github.com/umigv/controls_stack](https://github.com/umigv/controls_stack)
    - `cv_stack`: [https://github.com/umigv/cv_stack](https://github.com/umigv/cv_stack)
    - `ros_imu_bno055` (custom): [https://github.com/umigv/imu_driver](https://github.com/umigv/imu_driver)
    - `robot_localization`: [https://github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)
    - `cartographer`: [https://github.com/cartographer-project/cartographer](https://github.com/cartographer-project/cartographer)
    - `cartographer_ros`: [https://github.com/cartographer-project/cartographer_ros](https://github.com/cartographer-project/cartographer_ros)
    - `rosserial`: [https://github.com/ros-drivers/rosserial](https://github.com/ros-drivers/rosserial)
    - `velodyne`: [https://github.com/ros-drivers/velodyne](https://github.com/ros-drivers/velodyne)
    - `velodyne_simulator`: (for simulation) [https://bitbucket.org/DataspeedInc/velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator)

## Set up Cartographer

Comment out line 46 (`<!-- <depend>libabsl-dev</depend> -->`) in Cartographer package.xml as per [https://github.com/cartographer-project/cartographer_ros/issues/1726](https://github.com/cartographer-project/cartographer_ros/issues/1726).

Run these commands from the catkin workspace base folder (not `src`).

```bash
# Install prerequisites
sudo apt update
sudo apt install -y python3-wstool python3-rosdep ninja-build
sudo apt install -y liblua5.3-dev python3-sphinx libeigen3-dev
sudo apt install -y stow

# Install Cartographer dependencies
src/cartographer/scripts/install_debs_cmake.sh
src/cartographer/scripts/install_abseil.sh
src/marvin/scripts/install_proto3_fixed.sh

# Install ROS dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

## Build the Workspace

```bash
catkin build
source devel/setup.bash # run this command from the catkin workspace base folder (not src)
```

## Launch the Simulation

```bash
# Launch with sensors in RVIZ
roslaunch marvin world.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform')

# Launch with no RVIZ (for Cartgrapher)
roslaunch marvin world_no_rviz.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform')

# Launch Cartographer
roslaunch marvin cartographer_sim.launch 2> >(grep -v -E 'TF_REPEATED_DATA|buffer_core|lookupTransform|at line|^$')
```

## Launch the Robot

Make sure to plug in all of the following and check their ports:
- IMU (`/dev/ttyUSB0`)
- Encoder Arduino (`/dev/ttyACM0`)
- Motor Arduino (`/dev/ttyACM1`)
- Velodyne LiDAR (ethernet)
- ZED Camera (USB)

```bash
roslaunch marvin marvin.launch
```

## Control the Robot

For both simulation and the real robot.

```bash
# Control using keyboard
rosrun marvin teleop_twist_keyboard.py

# Control using Dualshock 4 controller
rosrun marvin velocity.py
```
