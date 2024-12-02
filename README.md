# HIWIN Robot ROS

[![License - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[ROS-Industrial](http://wiki.ros.org/Industrial) HIWIN metapackage.

## Contents
This repository follows branch naming aligned with ROS distributions.
- Stable branches: `indigo`, `kinetic`
- Development branches: `*-devel` (may be unstable)

## Compatibility
HRSS v3.3 or later.

## General Requirements
- **Operating System:** Ubuntu 20.04 LTS
- **ROS version:** Noetic Ninjemys

## Getting Started
1. **Install ros packages**
Follow the steps outlined in the [ROS Noetic installation instructions](https://wiki.ros.org/noetic/Installation).
2. **Source the ROS Environment**
```bash
source /opt/ros/noetic/setup.bash
```
3. **Create a ROS Workspace**
```bash
mkdir -p $HOME/catkin_ws/src
```
4. **Clone the Repository and Build**
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

## Usage
### :warning: **SAFETY FIRST**:warning:
*It is strongly recommended to test your code in simulation before using it on physical hardware.*

---

### Industrial robot simulator
To test the robot in a simulated environment:
```bash
$ roslaunch hiwin_ra610_1869_moveit_config moveit_planning_execution.launch sim:=true
```

---

### Real Robot Control
To connect to and control a physical robot:
```bash
$ roslaunch hiwin_driver hiwin_control.launch ra_type:=ra610_1869 robot_ip:=<robot ip>
```

---

### HRSS Offline Simulation
The **HIWIN Robot System Software (HRSS)** provides tools for offline simulation and testing.

#### System Requirements
| **Component**        | **Requirement**          |
|-----------------------|--------------------------|
| **Operating System**  | Microsoft Windows        |
| **Screen Resolution** | 1360x768 or higher       |

#### Steps
1. **Download HRSS Offline**  
   Obtain the software from the [HIWIN Download Center](https://www.hiwinsupport.com/download_center.aspx?pid=MAR).

2. **Start HRSS Offline**  
   - Select the robot model (e.g., `RA610-1869`).
   - Enable **EXT Mode**.
     ![HRSS Start](doc/images/hrss_offline_start.png)
     ![Remote Mode](doc/images/remote_mode.png)

3. **Connect to HRSS Simulator**  
   Launch the ROS node and replace `robot_ip` with your workstation's IP address:
   ```bash
   roslaunch hiwin_driver hiwin_control.launch ra_type:=ra610_1869 robot_ip:=<workstation ip>
   ```

4. **Control the Simulated Robot with MoveIt!**  
   ```bash
   roslaunch hiwin_ra610_1869_moveit_config moveit_planning_execution.launch sim:=false
   ```

---