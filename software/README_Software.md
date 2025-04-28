# Software
ECE Senior Design 2024-2025
Team 30

## Table of Contents
1. [Overview](#overview)
2. [Software Modules](#software-modules)
3. [Dependency Flow Chart](#dependency-flow-chart)
4. [Development and Build Tools](#development-and-build-tools)
5. [Installation Instructions](#installation-instructions)

---

## Overview
The `software` directory contains the implementation of key functionalities designed for the **SmoothOperator** project, a student-defined ECE Senior Design 2024-2025 project. This codebase is modular, allowing for easier integration, testing, and maintenance. The software primarily focuses on controlling hardware components, data processing, and interfacing with sensors and displays.

---

## Software Modules

### 1. **Navigation Stack**

The `NavigationStack` implements a 2D navigation system using ROS, which integrates odometry, sensor data, and goal poses to generate safe velocity commands for a mobile base. It includes the following key components:

### 2. **Sensor Data Node**

   - Purpose: Node that interfaces with the hardware sensors to provide real-time environmental and motion data.
   - Responsibilities:
     - LiDAR Node: Continuously publishes 2D laser scan data
representing the surrounding environment. The LiDAR scans in a
360 degree radius with a range of 12 meters.
     - Wheel Encoder & IMU Node: Reads and publishes raw odometry
data, including displacement, velocity, and orientation.

### 3. **Odometry Node**

   - Purpose: This node uses the data from the wheel encoders and IMU to calculate the
robot’s position and orientation over time via dead reckoning. The
differential drive kinematics model considers the physical wheelbase and
updates the robot’s estimated trajectory.

### 4. **Mapping Node**

   - Purpose: Creates a mapping of the environment.- Responsibilities:
     - Constructs a static 2D occupancy grid map using SLAM
(Simultaneous Localization and Mapping) techniques.
     - Our specific implementation uses the Rao-Blackwellized Particle Filter (RBPF), where
each particle represents a potential trajectory of the robot and carries its
own map of the environment.
     - The user/operator is able to annotate the map after it is produced to correct
minor blemishes or mark new environment obstacles.

### 5. **Localization Node**

   - Purpose: Estimates the robot’s current position with respect to the
pre-built map by using Monte Carlo Localization (AMCL).
   - Responsibilities:
     - Compares incoming LiDAR scans with the static map, which helps to align the
odometry-based coordinate frame (odom) with the map-based global
frame (map), allowing for accurate path planning and correction of
accumulated drift (known issue for dead reckoning). 

### 6. **Transform (TF) Nodes**

   - Purpose: TF nodes maintain a dynamic tree of coordinate frames between
the robot base, its sensors, and the world. These transforms enable
seamless integration of sensor data and motion commands by keeping all
information spatially aligned.

### 7. **Costmap Nodes**

   - Purpose: Maintain a representation of the environment by using an occupancy grid.
   - Responsibilities:
     - Global Costmap: Built using the static map produced by the
mapping node. It is used for long-range path planning and marks
static and annotated obstacles.
     - Local Costmap: Continuously updated with real-time LiDAR data
to account for dynamic obstacles (e.g., humans). It is used for
short-range navigation and obstacle avoidance.
     - Costmaps are implemented with a grid-based representation of space
where each cell in the grid holds a cost value indicating whether the area is
free, occupied, or unknown, and whether it’s near an obstacle. Both
costmaps also inflate a region around obstacles to create a buffer zone, so
the robot doesn’t plan paths too close to walls or objects.

### 8. **Path Planning Nodes**

   - Purpose: Generates most efficient paths according to a set of criterias.
   - Responsibilities:
     - Global Planner: Generates a high-level path from the robot’s current pose
to the goal, taking into account the global costmap to avoid static and
restricted areas. It uses Dijkstra’s algorithm to compute the shortest path
with cost-based weighting.
     - Local Planner: Implements the Dynamic Window Approach (DWA) which
considers multiple possible velocity commands and simulates the robot’s
motion over a short time step for each sampled command. It evaluates the
resulting trajectories using a cost function that balances multiple
objectives (how well it follows the global path, if it is avoiding obstacles
in the local costmap, and if the robot can fit through the path).

### 9. **Recovery Behavior Node**

   - Purpose: This node defines actions the robot should take when it becomes stuck or
cannot make forward progress. Examples include reversing slightly,
reattempting the global path from a new position, or clearing the local
costmap.

### 10. **Command Velocity Node**

   - Purpose: This node subscribes to the velocity outputs produced by the local planner
and sends them to the microcontroller (in our case, an Arduino) via a serial
interface. The microcontroller then translates these commands into motor
signals to drive the robot.

### 11. **User Interface Communication Node**

   - Purpose: This node facilitates communication between the robot and the user interface. It can
receive destination commands from the user interface and relays them to
the path planning nodes. This allows for dynamic goal-setting during
operation without restarting the navigation stack.

---

## Dependency Flow Chart

Below is the dependency flow chart showing the relationships between the modules:

![Software Diagram](https://github.com/TinkerSo/SmoothOperator/blob/main/images/SoftwareDiagram.png)

---

## Development and Build Tools

This project uses the following development and build tools:

- **Languages**:
  - Python 2.7 (standard for ROS1 Melodic)
  - C++11 (via GCC 7.5.0)

- **Build Tools**:
  - Catkin (build system for ROS packages)
  - CMake 3.10.2
  - Make 4.1

- **Libraries**:
  - ROS Navigation Stack (move_base, amcl, map_server, etc.) - ROS Melodic versions
  - Tf and tf2 libraries for coordinate frame transformations
  - Costmap_2d (for 2D costmaps used in planning)
  - Nav_msgs (for path planning and map data types)
  - Sensor_msgs (for LiDAR and odometry message types)
  - Geometry_msgs (for robot position and velocity messages)
  - Slam_gmapping (mapping)

---

## Installation Instructions

To set up the software stack from scratch, follow these steps:

### 1. Prerequisites
Ensure the following packages are installed on your system:
- Python 2.7
- C++11 (via GCC 7.5.0)
- CMake 3.10.2 or higher
- Make 4.1 or higher

**Ensure your Ubuntu system is version 18.04.**

### 2. Clone the Repository
```bash
git clone https://github.com/TinkerSo/SmoothOperator.git
```

### 3. Install ROS Melodic via Binary
![Link to Reference](https://wiki.ros.org/melodic/Installation/Ubuntu)

---
