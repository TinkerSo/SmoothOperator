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

### 1. **Sensor Data Node**
   - Purpose: Node that interfaces with the hardware sensors to provide real-time environmental and motion data.
   - Responsibilities:
     - LiDAR Node: Continuously publishes 2D laser scan data
representing the surrounding environment. The LiDAR scans in a
360 degree radius with a range of 12 meters.
     - Wheel Encoder & IMU Node: Reads and publishes raw odometry
data, including displacement, velocity, and orientation.
### 2. **Odometry Node**
   - Purpose: This node uses the data from the wheel encoders and IMU to calculate the
robot’s position and orientation over time via dead reckoning. The
differential drive kinematics model considers the physical wheelbase and
updates the robot’s estimated trajectory.
### 3. **Mapping Node**
   - Purpose: Creates a mapping of the environment.
   - Responsibilities:
     - Constructs a static 2D occupancy grid map using SLAM
(Simultaneous Localization and Mapping) techniques.
     - Our specific
implementation uses the Rao-Blackwellized Particle Filter (RBPF), where
each particle represents a potential trajectory of the robot and carries its
own map of the environment.
     - The
user/operator is able to annotate the map after it is produced to correct
minor blemishes or mark new environment obstacles.
### 4. **Localization Node**
   - Purpose: Estimates the robot’s current position with respect to the
pre-built map by using Monte Carlo Localization (AMCL).
   - Responsibilities:
     - Compares
incoming LiDAR scans with the static map, which helps to align the
odometry-based coordinate frame (odom) with the map-based global
frame (map), allowing for accurate path planning and correction of
accumulated drift (known issue for dead reckoning). 

### 5. **Transform (TF) Nodes**
   - Purpose: TF nodes maintain a dynamic tree of coordinate frames between
the robot base, its sensors, and the world. These transforms enable
seamless integration of sensor data and motion commands by keeping all
information spatially aligned.

### 6. **Costmap Nodes**
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


---

## Dependency Flow Chart

Below is the dependency flow chart showing the relationships between the modules:

```

```

---

## Development and Build Tools

This project uses the following development and build tools:

- **Languages**:
  - Python 3.8 or higher
  - C++ (GCC Compiler)
  - Assembly

- **Build Tools**:
  - Make 4.2.1
  - CMake 3.21.2

- **Libraries**:
  - OpenCV 4.5.2 (for image processing)
  - NumPy 1.21.0 (for numerical computations)
  - PySerial 3.5 (for serial communication)

---

## Installation Instructions

To set up the software stack from scratch, follow these steps:

### 1. Prerequisites
Ensure the following packages are installed on your system:
- Python 3.8 or higher
- GCC 9.3.0 or higher
- CMake 3.21.2 or higher
- Make 4.2.1 or higher

### 2. Clone the Repository
```bash
git clone https://github.com/TinkerSo/SmoothOperator.git
cd SmoothOperator/software
```

### 3. Install Python Dependencies
Create a virtual environment and install the required Python packages:
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 4. Build C++ and Assembly Components
Run the following commands to compile the C++ and Assembly code:
```bash
mkdir build
cd build
cmake ..
make
```

### 5. Run the Software
To start the main program:
```bash
python main.py
```

---
