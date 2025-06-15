# robot-vacuumAdd commentMore actions

*A lightweight robot-vacuum model built in Fusion 360, delivered with ROS‑ready components for quick SLAM integration.*

---

## Cleaning Machine Description 📦

This repository contains **only** the 3‑D model (URDF/Xacro), Gazebo plugins, and RViz layouts for ROS Noetic + Gazebo 11.  
Mapping / localization / navigation should live in a separate package (e.g. `cleaning_machine_navigation`).

---

## Requirements
| Software | Version |
|----------|---------|
| **Ubuntu** | 20.04 LTS |
| **ROS** | Noetic |
| **Gazebo** | 11 |
| Extra deps | `gazebo_ros` · `robot_state_publisher` · `joint_state_publisher_gui` |

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs                  ros-noetic-robot-state-publisher                  ros-noetic-joint-state-publisher-gui
```

---

## Installation
```bash
# 1) create a catkin workspace (skip if you already have one)
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# 2) clone this repo
# remember to go inside catkin_ws/src
cd catkin_ws/src
git clone git@github.com:Jack23162329/robot-vacuum.git

# 3) build & source
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Quick Start
| Action | Command |
|--------|---------|
| **Launch Gazebo + RViz** | `roslaunch cleaning_machine_description gazebo.launch` |
| **View URDF only (no Gazebo)** | `roslaunch cleaning_machine_description display.launch` |
| **Start wheel & brush controllers** | `roslaunch cleaning_machine_description controller.launch` |

**RViz layouts**  
* `rviz/urdf.rviz` — model‑only  
* `rviz/urdf_slam.rviz` — adds **LaserScan** and `/map` displays for future SLAM

---

## Directory Tree
```
cleaning_machine_description/
├── launch/
│   ├── gazebo.launch          # Gazebo + model
│   ├── display.launch         # RViz only
│   ├── controller.launch      # diff‑drive & brush control
│   └── controller.yaml        # PID / topics
├── meshes/                    # STL meshes
├── urdf/
│   ├── cleaning_machine.xacro
│   ├── cleaning_machine.gazebo
│   ├── cleaning_machine.trans
│   └── materials.xacro
├── rviz/
│   ├── urdf.rviz
│   └── urdf_slam.rviz
└── worlds/
    ├── turtlebot3_house.world
    └── turtlebot3_world.world
```

---

## Feature Overview
| Feature | File / Plugin | Notes |
|---------|---------------|-------|
| **Four‑wheel differential drive** | `urdf/cleaning_machine.gazebo` (`libgazebo_ros_diff_drive`) | left joint = **Revolute 7** · right joint = **Revolute 8** |
| **Dual rotating brushes** | `joint_velocity_controller` | controlled via **`/brush_cmd`** (`geometry_msgs/Twist`) |
| **2‑D Lidar** | Hokuyo plugin → **`/scan`** | sensor frame `lidar_link` fixed to `base_link` |
| **RViz layouts** | `rviz/urdf*.rviz` | ready‑made **Grid / Laser / Map** displays |
| **Gazebo demo world** | `worlds/turtlebot3_house.world` | multi‑room interior with obstacles |

---

## FAQ / Troubleshooting
| Issue | Fix |
|-------|-----|
| RViz shows **“No transform from `left_clean_1`”** | Enable `<publishJointStates>true>` in `cleaning_machine.gazebo`, or change the brush `<joint>` to `type="fixed"`. |
| Robot does **not** move in Gazebo | Make sure `controller.launch` is running **and** `/cmd_vel` is publishing. |
| Model looks off‑scale | Check STL `<scale>` and verify `wheelDiameter` / `wheelSeparation`. |

---

## License
Distributed under the MIT License – see `LICENSE` for details.
# robot-vacuumAdd commentMore actions
# robot-vacuum
