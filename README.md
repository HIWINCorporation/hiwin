# HIWIN Robot

[![License - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[ROS-Industrial](http://wiki.ros.org/Industrial) HIWIN metapackage.

# Contents
Branch naming follows the ROS distribution they are compatible with. -devel branches may be unstable. Releases are made from the distribution branches (indigo, kinetic).

# Installation
### Building from Source
```bash
# change to the root of the Catkin workspace
$ cd $HOME/catkin_ws

# retrieve the sources
$ git clone -b noetic-devel https://github.com/HIWINCorporation/hiwin_ros.git src/

# check build dependencies. Note: this may install additional packages,
# depending on the software installed on the machine
$ rosdep update

# be sure to change 'noetic' to whichever ROS release you are using
$ rosdep install --from-paths src/ --ignore-src --rosdistro noetic

# build the workspace (using catkin_tools)
$ catkin_make

# activate this workspace
$ source $HOME/catkin_ws/devel/setup.bash
```

# Usage
## Industrial robot simulator
```bash
$ roslaunch hiwin_ra610_1869_moveit_config moveit_planning_execution.launch
```

## [HIWIN Robot System Software Offline](https://www.hiwinsupport.com/download_center.aspx?pid=MAR)

The HIWIN Robot System Software(HRSS) controls all basic functions for the robot.

Before connecting to the actual robot, we can use HRSS Offline as a substitute to simulate the real robot.

![Alt text](doc/images/hiwin_download_center.png)

### Workstation PC

| Minimum System Requirements| |
| ------------- | ------------- |
| Operating System  | Microsoft Windows  |
|Resolution| 1360 * 768 above


### Getting started

Start HRSS Offline, select the type of robot, and press the EXT mode.

<img src="doc/images/hrss_offline_start.png" width="600" />

<img src="doc/images/remote_mode.png" width="600" />

Start the ROS driver and connect to HRSS Offline.
```bash
$ roslaunch hiwin_driver ra6_bringup.launch ra_type:=ra610_1869 robot_ip:=<workstation ip>
```

You can use MoveIt! to control the robot.

```bash
$ roslaunch hiwin_ra610_1869_moveit_config moveit_planning_execution.launch sim:=false
```