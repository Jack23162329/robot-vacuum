# robot-vacuumAdd commentMore actions

*A lightweight robot-vacuum model built in Fusion 360, delivered with ROS‑ready components for quick SLAM integration.*
This repository contains the 3‑D model (URDF/Xacro), Gazebo plugins, and RViz layouts for ROS Noetic + Gazebo 11. where we are able to create map by moving robot-vacuum 
Where Gazebo demo world is directory copy by turtlebot3. I didn't make it by myself.

---

# Requirements 👍
| Software | Version |
|----------|---------|
| **Ubuntu** | 20.04 LTS |
| **ROS** | Noetic |
| **Gazebo** | 11 |
| Extra deps | `gazebo_ros` · `robot_state_publisher` · `joint_state_publisher_gui` |

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs \
                 ros-noetic-robot-state-publisher \
                 ros-noetic-joint-state-publisher-gui \
                 ros-noetic-slam-gmapping \
                 ros-noetic-map-server
```

---

# Installation 👾
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
# Robot appearance Demo Video ▶️

https://github.com/user-attachments/assets/d0a98370-3f69-4563-8969-e6da83003fc7

# Quick Start 😊
Launch Gazebo + RViz + SLAM 
```bash 
roslaunch cleaning_machine_description gazebo.launch
```

> **⚠️ IMPORTANT – TELEOP CONTROL**  
> The `teleop_twist_keyboard.py` node is launched **inside `gazebo.launch`**.  
> Keep **the same terminal window** focused and drive the robot with `I J K L` and other keyboard keys nearby

## Demo Video ▶️

https://youtu.be/Kb1GFwbw0Os 

![Final Map](https://github.com/user-attachments/assets/025e07ae-5d88-4a32-a3f0-c17acb991360)

View URDF by rviz only (no Gazebo)
```bash
roslaunch cleaning_machine_description display.launch
```

## RViz layouts
* `rviz/urdf_slam.rviz` — adds **LaserScan** and `/map` displays for future SLAM (default settings)
* `rviz/urdf.rviz` — model‑only  

---

# Directory Tree 🌲
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

## Feature Overview 😸
| Feature | File / Plugin | Notes |
|---------|---------------|-------|
| **Four‑wheel differential drive** | `urdf/cleaning_machine.gazebo` (`libgazebo_ros_diff_drive`) | left joint = **Revolute 7** · right joint = **Revolute 8** |
| **Dual rotating brushes** | `joint_velocity_controller` | controlled via **`/brush_cmd`** (`geometry_msgs/Twist`) |
| **2‑D Lidar** | Hokuyo plugin → **`/scan`** | sensor frame `lidar_link` fixed to `base_link` |
| **RViz layouts** | `rviz/urdf*.rviz` | ready‑made **Grid / Laser / Map** displays |
| **Gazebo demo world** | `worlds/turtlebot3_house.world` | multi‑room interior with obstacles, where I just copy the maps from turtlebot3 |

