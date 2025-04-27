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

###  1. **Navigation Stack**

The `NavigationStack` implements a 2D navigation system using ROS, which integrates odometry, sensor data, and goal poses to generate safe velocity commands for a mobile base. It includes the following key components:

- **Global and Local Planners**: Interfaces (`BaseGlobalPlanner`, `BaseLocalPlanner`) and implementations for path planning.
- **SLAM Integration**: Modules, such as `slam_gmapping`, for simultaneous localization and mapping.
- **NavFn Library**: Implements Dijkstra-based pathfinding algorithms.
- **Hector SLAM**: Provides trajectory generation and map creation capabilities.
- **Map Server**: A utility to manage and serve map data.

Here’s a combined and summarized section for the `SmoothOperatorUI` directory, including details about `faceUI.py`, suitable for a `README.md` file:

---

### 2. **SmoothOperatorUI**

The `SmoothOperatorUI` directory contains resources and scripts for managing the user interface and related utilities of the SmoothOperator system. It is built using Python and the Kivy framework, with additional integrations for custom widgets, audio playback, and server communication.

- **faceUI.py**: The core GUI script that handles screen transitions, animations, audio synchronization, and WebSocket communication.  
  - **Theme and Configuration**: Implements a consistent application theme and sets global configurations like server URLs and window size.
  - **Custom Widgets**: Includes components such as `RoundedButton` for customizable buttons, `HeaderBar` for navigation, and facial animation widgets like `MouthWidget`.
  - **Screens**: Manages user interactions across multiple screens:
    - **FaceScreen**: Displays animated facial expressions and handles commands via WebSocket.
    - **GoalReachedScreen**: Confirms task completion and provides unloading controls.
    - **ConnectScreen**: Enables device connectivity using a passcode.
    - **MenuScreen**: Acts as a navigation hub for selecting features.
    - **ManualControlScreen**: Allows manual control of the robot.
    - **QRScreen**: Scans boarding passes via webcam and processes QR codes.
    - **PostScanScreen**: Displays QR scan details and user confirmation options.
    - **HelpScreen**: Offers user instructions for using the system.
  - **SoundManager**: Plays audio files for events like greetings, confirmations, and alerts.
- **QRCodeGenerator.py**: A script for generating QR codes for use in the system.
- **audio/**: A directory containing audio assets used in the interface.
- **terminalQRCodes/**: Stores terminal-based QR code outputs.
- **testFiles/**: Contains test scripts and resources for UI development.

### 2. **Example**
   - Purpose: 
   - Responsibilities:
     - 1
     - 2
     - 3
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
