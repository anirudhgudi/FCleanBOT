# FCleanBOT

A ROS-based project demonstrating an autonomous floor-cleaning robot that maps its environment, plans coverage paths, and detects/relocates objects in its path using a TurtleBot3 Waffle Pi with an OpenManipulator-X arm. The robot is simulated in Gazebo, using SLAM to generate an occupancy grid map, and Python scripts to execute a boustrophedon coverage algorithm and a simple obstacle-handling routine.

## Table of Contents
1. [Overview](#overview)  
2. [Features](#features)  
3. [Requirements](#requirements)  
4. [Quick Start](#quick-start)  
5. [Usage](#usage)  
6. [File Structure](#file-structure)  
7. [Contributing](#contributing)  

---

## 1. Overview

This repository contains launch files, Python scripts, and configuration files to simulate a TurtleBot3 Waffle Pi equipped with an OpenManipulator-X. The robot autonomously navigates, maps the environment, and can pick or relocate objects detected via LiDAR.

---

## 2. Features
- **Gazebo Simulation** of TurtleBot3 Waffle Pi + OpenManipulator-X  
- **2D SLAM** with the `map_server` for occupancy grids  
- **Boustrophedon Coverage** algorithm for systematic floor cleaning  
- **Object Detection and Manipulation** via simple Inverse Kinematics (IK)  
- **Integration** with `roslaunch` for quick startup

---

## 3. Requirements
- **Ubuntu 20.04** or similar (ROS Noetic recommended)
- **TurtleBot3 packages** (`turtlebot3`, `turtlebot3_manipulation_*`, etc.)
- **MoveIt!** for motion planning
- **Gazebo** (version matching your ROS distribution)
- **Python 3**  
- **map_server** for serving the occupancy grid map

Install typical dependencies (adjust for your ROS version/distro):
```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-manipulation \
                     ros-noetic-turtlebot3-manipulation-gazebo \
                     ros-noetic-map-server ros-noetic-moveit
```

---

## 4. Quick Start

1. **Clone** this repository into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
  git clone https://github.com/anirudhgudi/FCleanBOT.git
     ```

2. **Build** the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Verify** that you have:
   - A `home.pgm` map in your repository  
   - `coverage.py` and `obs_manipu.py` scripts  
   - The `turtlebot3_manipulation_*` packages installed

---

## 5. Usage

Below is the **typical sequence** of commands to bring up the robot in Gazebo, load the map, start MoveIt! for motion planning, and run the coverage and obstacle-handling scripts.

1. **Launch the Gazebo simulation** for TurtleBot3 + OpenManipulator:
   ```bash
   roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
   ```
2. **Run the map server** to provide the occupancy grid:
   ```bash
   rosrun map_server map_server home.pgm
   ```
3. **Bring up** the robot manipulation layers:
   ```bash
   roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
   ```
4. **Start MoveIt!** for arm motion planning:
   ```bash
   roslaunch turtlebot3_manipulation_moveit_config move_group.launch
   ```
5. **Execute the coverage script** (boustrophedon path planning):
   ```bash
   python coverage.py
   ```
6. **Run the obstacle-handling script** (pick-and-place routine):
   ```bash
   python obs_manipu.py
   ```

> **Tip**: These commands typically require separate terminals, or use a tool like `tmux` to organize your sessions. Make sure each terminal has sourced your workspace (`source ~/catkin_ws/devel/setup.bash`).

---

## 6. File Structure

A simplified directory layout:

```
fcleanbot/
├── launch/
│   ├── turtlebot3_manipulation_gazebo.launch
│   ├── turtlebot3_manipulation_bringup.launch
│   └── ...
├── maps/
│   └── home.pgm
├── scripts/
│   ├── coverage.py      # Boustrophedon coverage
│   ├── obs_manipu.py    # Obstacle detection & manipulation
├── README.md
└── ...
```

- **`launch/`**: Contains the main ROS launch files.  
- **`maps/`**: Holds the occupancy grid (`.pgm`) for `map_server`.  
- **`scripts/`**: Python nodes for coverage path planning and object-handling.  
- **`turtlebot3_manipulation_moveit_config/`**: MoveIt! config files.

---

## 7. Contributing

Contributions are welcome! Feel free to open an issue or submit a pull request for:
- Bug fixes
- Additional features (e.g., advanced vision-based object recognition)
- Documentation improvements



