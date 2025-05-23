# Software
ECE Senior Design 2024-2025
Team 30

## Table of Contents
- [Overview](#overview)
- [Software Modules](#software-modules)
- [Dependency Flow Chart](#dependency-flow-chart)
- [Development and Build Tools](#development-and-build-tools)
- [Installation Instructions](#installation-instructions)

---

## Overview
The `software` directory contains the implementation of key functionalities designed for the **SmoothOperator** project, a student-defined ECE Senior Design 2024-2025 project. This codebase is modular, allowing for easier integration, testing, and maintenance. The software primarily focuses on controlling hardware components, data processing, and interfacing with sensors and displays.

---

## Software Modules

### 1.0 **Navigation Stack**

The `NavigationStack` implements a 2D navigation system using ROS, which integrates odometry, sensor data, and goal poses to generate safe velocity commands for a mobile base. It includes the following key components:

#### 1.1 **Sensor Data Node**

   - Purpose: Node that interfaces with the hardware sensors to provide real-time environmental and motion data.
   - Responsibilities:
     - LiDAR Node: Continuously publishes 2D laser scan data
representing the surrounding environment. The LiDAR scans in a
360 degree radius with a range of 12 meters.
     - Wheel Encoder & IMU Node: Reads and publishes raw odometry
data, including displacement, velocity, and orientation.

#### 1.2 **Odometry Node**

   - Purpose: This node uses the data from the wheel encoders and IMU to calculate the
robot’s position and orientation over time via dead reckoning. The
differential drive kinematics model considers the physical wheelbase and
updates the robot’s estimated trajectory.

#### 1.3 **Mapping Node**

   - Purpose: Creates a mapping of the environment.- Responsibilities:
     - Constructs a static 2D occupancy grid map using SLAM
(Simultaneous Localization and Mapping) techniques.
     - Our specific implementation uses the Rao-Blackwellized Particle Filter (RBPF), where
each particle represents a potential trajectory of the robot and carries its
own map of the environment.
     - The user/operator is able to annotate the map after it is produced to correct
minor blemishes or mark new environment obstacles.

#### 1.4 **Localization Node**

   - Purpose: Estimates the robot’s current position with respect to the
pre-built map by using Monte Carlo Localization (AMCL).
   - Responsibilities:
     - Compares incoming LiDAR scans with the static map, which helps to align the
odometry-based coordinate frame (odom) with the map-based global
frame (map), allowing for accurate path planning and correction of
accumulated drift (known issue for dead reckoning). 

#### 1.5 **Transform (TF) Nodes**

   - Purpose: TF nodes maintain a dynamic tree of coordinate frames between
the robot base, its sensors, and the world. These transforms enable
seamless integration of sensor data and motion commands by keeping all
information spatially aligned.

#### 1.6 **Costmap Nodes**

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

#### 1.7 **Path Planning Nodes**

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

#### 1.8 **Recovery Behavior Node**

   - Purpose: This node defines actions the robot should take when it becomes stuck or
cannot make forward progress. Examples include reversing slightly,
reattempting the global path from a new position, or clearing the local
costmap.

#### 1.9 **Command Velocity Node**

   - Purpose: This node subscribes to the velocity outputs produced by the local planner
and sends them to the microcontroller (in our case, an Arduino) via a serial
interface. The microcontroller then translates these commands into motor
signals to drive the robot.

#### 1.10 **User Interface Communication Node**

   - Purpose: This node facilitates communication between the robot and the user interface. It can
receive destination commands from the user interface and relays them to
the path planning nodes. This allows for dynamic goal-setting during
operation without restarting the navigation stack.

---

### 2.0 **Onboard UI System**

The `SmoothOperatorApp` is a Kivy-based Python application running on a touchscreen device (e.g., Jetson Nano with display) that serves as the primary onboard interface for the SmoothOperator™ robot. It handles user interactions, boarding pass scanning, robot control, and communication with the server over HTTP and WebSocket protocols.

#### 2.1 **FaceScreen**

- **Purpose:** The default idle screen that visually represents the robot’s face and awaits user interaction or remote commands.
- **Responsibilities:**
  - Renders animated facial expressions (blinking eyes, animated/static mouth).
  - Displays a floating help button (launches HelpScreen).
  - Starts a persistent WebSocket connection to listen for movement commands and goal completion messages.
  - Transitions to `MenuScreen` upon touch.
  - Handles “Goal Reached” signal by switching to `GoalReachedScreen`.
  - Animates eyes and mouth in response to motion or speech playback.
  - Plays relevant audio via `SoundManager`.

#### 2.2 **MenuScreen**

- **Purpose:** Central navigation hub for all available onboard features.
- **Responsibilities:**
  - Provides access to: 
    - Manual Robot Control
    - Load Luggage
    - QR Code Boarding Pass Scan
    - Connect to App
    - Help
  - Renders a set of `RoundedButton` widgets in a vertical layout.
  - Manages screen transitions via callback functions to respective modules.

#### 2.3 **ManualControlScreen**

- **Purpose:** Allows users to control the robot manually using on-screen direction and speed buttons.
- **Responsibilities:**
  - Sends motor commands (`w`, `a`, `s`, `d`, `x`) to the backend over WebSocket.
  - Allows speed selection via `L`, `M`, `H` commands.
  - Displays directional buttons in a grid layout.
  - Highlights the selected speed using color changes.
  - Triggers sound feedback based on motion type (e.g., `move_forward`, `turn_left`).

#### 2.4 **QRScreen**

- **Purpose:** Captures and decodes boarding pass QR codes via webcam feed using OpenCV and Pyzbar.
- **Responsibilities:**
  - Initializes a camera feed using `cv2.VideoCapture`.
  - Decodes QR codes in real time and parses flight data from embedded JSON.
  - Shows live feedback below the camera (e.g., “Please Scan” or error messages).
  - Animates green flash upon successful scan.
  - Transitions to `PostScanScreen` with extracted payload (passenger and flight info).
  - Plays specific professor-themed sounds if matched by name.

#### 2.5 **PostScanScreen**

- **Purpose:** Displays decoded boarding pass details and asks user to confirm autonomous delivery.
- **Responsibilities:**
  - Renders parsed passenger, flight, and gate information in markup-rich label.
  - Offers “Yes” and “No” buttons:
    - Yes: Sends data to server via HTTP POST, transitions to `FaceScreen`.
    - No: Discards data and returns to `MenuScreen`.
  - Handles screen transition using `CardTransition`.

#### 2.6 **GoalReachedScreen**

- **Purpose:** Confirms if the user has completed their trip after arriving at the destination.
- **Responsibilities:**
  - Triggered by `"Goal Reached"` WebSocket message from backend.
  - Displays terminal/gate info and prompts user with "Are you finished?".
  - If “Yes”: Reveals unloading controls (Up, Down, Confirm).
  - Sends final payload confirming trip completion.
  - If “No”: Returns to `MenuScreen`.

#### 2.7 **ConnectScreen**

- **Purpose:** Enables mobile app to pair with SmoothOperator via a passcode.
- **Responsibilities:**
  - Displays a randomly generated 4-digit code.
  - Sends the passcode to the server via HTTP POST.
  - Listens for `AUTH_SUCCESS` WebSocket message to confirm pairing.
  - Transitions to confirmation screen and allows return to `FaceScreen`.


#### 2.8 **LoadLuggageScreen**

- **Purpose:** Provides direct control over the lift motor to load items into the robot.
- **Responsibilities:**
  - Displays “Up” and “Down” buttons to raise or lower the loading platform.
  - Sends lift commands over WebSocket (`+`, `-`, `=`).
  - Used before starting autonomous or manual delivery.

#### 2.9 **HelpScreen**

- **Purpose:** Offers guidance on how to operate SmoothOperator.
- **Responsibilities:**
  - Static instructions on navigating the UI and using key features.
  - Markup formatting for clarity (e.g., bolded section titles).
  - Accessible from both `FaceScreen` and `MenuScreen`.

#### 2.10 **Audio System**

All audio used in the onboard UI is managed by the `SoundManager` class and played using `pygame.mixer`. Sound files are stored in the `/audio` directory and include voice prompts, feedback cues, and themed greetings for specific passengers.

---

### 3.0 **Mobile App UI**

The `UserUI` is a React Native application that allows users to remotely interact with the SmoothOperator™ system. It consists of intuitive interfaces designed to authenticate users and control the robot in real time through a WebSocket connection. This module is optimized for accessibility, responsiveness, and safety.

#### 3.1 **Passcode Page**

- **Purpose:** Provides a secure gateway to access robot controls by requiring a valid 4-digit passcode, displayed on the onboard SmoothOperator screen.
- **Responsibilities:**
  - Displays a virtual numeric keypad for passcode input.
  - Validates the entered passcode by sending it over a WebSocket connection to the SmoothOperator onboard system.
  - Displays success or failure messages based on validation response.
  - Offers a reset or backspace option for correcting mistyped digits.
  - Prevents further interaction with control features until correct authentication is received.

#### 3.2 **Control Pad Page**

- **Purpose:** Allows real-time remote control of the robot via directional and action-based buttons.
- **Responsibilities:**
  - Displays an on-screen directional pad with `Forward`, `Reverse`, `Left`, `Right`, and `Stop` buttons.
  - Sends single-character commands (`w`, `a`, `s`, `d`, `x`) to the onboard system over WebSocket for navigation.
  - Displays visual feedback indicating the current movement or status (e.g., “Moving Forward”).
  - Optionally includes speed selection (Low/Medium/High) if implemented.
  - Includes a large, visually distinct “STOP” button for immediate safety.
  - Allows the user to return to the authentication screen or exit the session.

---

### 4.0 **Arduino Firmware**

The Arduino microcontroller runs low-level firmware that controls the SmoothOperator™ robot’s motors, LEDs, lift system, ultrasonic sensors, and bump switches. It serves as the interface between high-level commands received from the onboard computer and the hardware peripherals, translating movement and control instructions into real-time actuation and feedback.

#### 4.1 **Motion Control System**

- **Purpose:** Drives the left and right wheels via RoboClaw motor drivers based on velocity and angular rotation commands.
- **Responsibilities:**
  - Parses serial input in the format: `Vx Vy Vtheta lift`, with each float formatted to exactly three decimal places.
  - Converts linear and angular velocity commands using tank drive kinematics.
  - Calculates and sends appropriate `QPPS` (quadrature pulses per second) to RoboClaw’s `M1` and `M2` channels.
  - Updates NeoPixel LEDs to indicate motion (green = moving, red = idle).
  - Implements encoder feedback to compute and send displacement and velocity data over `Serial1`.

#### 4.2 **Lift System Control**

- **Purpose:** Manages vertical actuation of the luggage platform using an L298N H-bridge motor driver.
- **Responsibilities:**
  - Accepts lift commands as a float (`>0` for up, `<0` for down, `0` for stop).
  - Drives digital output pins (`IN1`, `IN2`, `ENA`) to set motor direction and PWM speed.
  - Stops the motor automatically when no lift command is present.
  - Uses analog PWM to control lift speed.

#### 4.3 **Obstacle Detection System**

- **Purpose:** Prevents collisions by monitoring surrounding obstacles using bump switches and ultrasonic sonar.
- **Responsibilities:**
  - Monitors four bumper inputs (`FRONT_BUMP`, `BACK_BUMP`, `LEFT_BUMP`, `RIGHT_BUMP`).
  - Reads sonar distance using the NewPing library (TRIG/ECHO pins).
  - Immediately halts motor commands and sets LEDs to red when any obstacle is detected.

#### 4.4 **LED Feedback System**

- **Purpose:** Provides visual cues for robot status using a NeoPixel LED strip.
- **Responsibilities:**
  - `Green`: Robot is actively moving.
  - `Red`: Robot is idle or has detected an obstacle.
  - Controlled using Adafruit NeoPixel library (`LED_PIN` with 300 RGB LEDs).
  - Color and brightness updated in real-time based on robot state.

#### 4.5 **Serial Communication Protocol**

- **Purpose:** Enables high-level systems (e.g., Jetson Nano) to send commands to the Arduino for execution.
- **Responsibilities:**
  - Listens to `Serial` input formatted as: `Vx Vy Vtheta lift`.
  - Validates input for correct spacing and 3-decimal precision.
  - Outputs encoder-based speed and displacement metrics over `Serial1` for real-time monitoring or logging.

#### 4.6 **Roboclaw Motor Driver Integration**

- **Purpose:** Interfaces with RoboClaw over software serial to control high-power DC motors with encoder feedback.
- **Responsibilities:**
  - Initializes velocity PID parameters using `SetM1VelocityPID` and `SetM2VelocityPID`.
  - Resets encoders on startup.
  - Reads encoder ticks and speeds to derive physical movement characteristics (velocity and displacement).
  - Uses RoboClaw's M1 (right) and M2 (left) for directional drive control.

#### 4.7 **Startup Behavior**

- **Purpose:** Safely initializes all components on boot.
- **Responsibilities:**
  - Starts serial communication (`Serial` and `Serial1`).
  - Initializes LED strip with default red color.
  - Configures motor pins and RoboClaw PID parameters.
  - Resets encoders and ensures robot is stationary.
  - Prints status messages to `Serial`.

---

### 5.0 **Node.js Server**

The Node.js server acts as a communication bridge between the onboard system, Arduino microcontroller, React Native app, and external ROS processes. It manages serial I/O, real-time WebSocket communication, HTTP endpoints, and command translation logic to ensure smooth bidirectional data flow across all layers of the SmoothOperator™ system.

---

#### 5.1 **Serial Communication with Arduino**

- **Purpose:** Sends real-time movement and lift commands to the Arduino and receives sensor feedback.
- **Responsibilities:**
  - Opens a serial connection with the Arduino (`/dev/ttyACM0`) at `9600` baud.
  - Uses the `ReadlineParser` to process newline-terminated UART data.
  - Forwards Arduino data to all active WebSocket clients in UTF-8 format.
  - Handles transmission reliability with `.drain()` calls after writes.

#### 5.2 **WebSocket Server**

- **Purpose:** Enables real-time bidirectional messaging between the onboard UI, React Native app, and backend logic.
- **Responsibilities:**
  - Hosts a WebSocket server on port `3000`.
  - Broadcasts messages to all clients, including:
    - Movement commands (`w`, `a`, `s`, `d`, etc.)
    - Speed mode updates (`L`, `M`, `H`)
    - Authentication status (`AUTH_SUCCESS`)
    - System states (e.g., `Goal Reached`)
  - Relays movement commands to the Arduino with calculated speed-based velocity commands.
  - Applies a 50ms delay before sending commands to prevent serial overload.

#### 5.3 **Command Translation & Speed Scaling**

- **Purpose:** Maps symbolic movement commands to structured velocity commands expected by the Arduino.
- **Responsibilities:**
  - Maintains a `baseCommands` object with templates using `{speed}` placeholders.
  - Applies current speed mode (`L`, `M`, `H`) from global state to calculate real-time values.
  - Translates incoming WebSocket commands into serial instructions (e.g., `w` → `"0.100 1.000 0.000 0.000"`).
  - Supports both onboard and remote directional modes (e.g., `w` vs `wr`).

#### 5.4 **ROS Integration Endpoint**

- **Purpose:** Forwards QR code data to a local ROS node for routing and navigation.
- **Route:** `POST /api/QR`
- **Responsibilities:**
  - Accepts payload fields: `name`, `flight`, `to`, `from`, `dep_time`, `terminal`, `gate`.
  - Validates required fields.
  - Forwards `terminal` and `gate` to the ROS endpoint at `http://localhost:8000/api/QR`.
  - Responds to the onboard app with forwarding confirmation or error.

#### 5.5 **ROS Motion Command Endpoint**

- **Purpose:** Accepts motion commands from ROS and relays them to the Arduino.
- **Route:** `POST /api/ros`
- **Responsibilities:**
  - Validates that data has four space-separated float values.
  - Caps angular velocity `Vtheta` to `0.150` if exceeded.
  - Sends velocity string to Arduino (via UART) after formatting to 3-decimal precision.
  - Performs binning logic to determine discrete movement direction (`w`, `a`, `s`, `d`, `x`) based on velocity thresholds.
  - Broadcasts binned direction via WebSocket for Kivy UI synchronization.

#### 5.6 **Passcode Authentication System**

- **Purpose:** Secures access to robot controls from the mobile app via a dynamic passcode.
- **Routes:**
  - `POST /api/connect` — Robot sends and sets the current session's passcode.
  - `POST /api/authenticate` — Mobile app sends user-entered passcode for validation.
- **Responsibilities:**
  - Stores the current passcode in memory (`currentPasscode`).
  - Verifies app passcode against stored value and responds with success/failure.
  - On successful authentication, broadcasts `"AUTH_SUCCESS"` via WebSocket.

---

## Dependency Flow Chart

Below is the dependency flow chart showing the relationships between the modules:

![Software Diagram](https://github.com/TinkerSo/SmoothOperator/blob/main/images/SoftwareDiagram.png)

*Navigation Stack Diagram*

![On-board UI Diagram](https://github.com/user-attachments/assets/d8726497-cb89-449b-9d33-eb0d3d1307cb)

*On-board Robot UI Diagram*

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
The instructions below are directly from the [ROS wiki site](https://wiki.ros.org/melodic/Installation/Ubuntu)

   #### 3.1 Configure your Ubuntu repositories
   - Configure your Ubuntu repositories to allow "restricted", "universe", and "multiverse". You can [follow the Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.

   #### 3.2 Setup your sources.list
   - Setup your computer to accept software from packages.ros.org.
   ```
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

   #### 3.3 Setup your keys
   ```
   sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

   #### 3.4 Installation
   ```
   sudo apt update
   sudo apt install ros-melodic-desktop-full
   ```

   #### 3.5 Environment setup
   - It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
   ```
   echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
   ```
   - If you have more than one ROS distribution installed, ~/.bashrc must only source the setup.bash for the version you are currently using.
   - If you just want to change the environment of your current shell, instead of the above you can type:
   ```
   source /opt/ros/melodic/setup.bash
   ```

   #### 3.6 Dependencies for building packages
   - To install tools and other dependencies for building ROS packages, run:
   ```
   sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
   ```

   #### 3.7 Initialize rosdep
   - Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
   ```
   sudo apt install python-rosdep
   ```
   - With the following, you can initialize rosdep
   ```
   sudo rosdep init
   rosdep update
   ```
   
---
