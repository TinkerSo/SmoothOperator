# SmoothOperator
ECE Senior Design 2024–2025  
Team 30

# Engineering Addendum

## Table of Contents
1. [Project Overview](#project-overview)
2. [Current State of the Project](#current-state-of-the-project)
3. [System Architecture Overview](#system-architecture-overview)
4. [Gotchas and Important Notes for Future Teams](#gotchas-and-important-notes-for-future-teams)
5. [Setup and Quick Start Instructions](#setup-and-quick-start-instructions)
6. [Future Recommendations](#future-recommendations)
7. [Additional Documentation](#additional-documentation)
8. [Team Members](#team-members)

---

## Project Overview

![SmoothOperator prototype fully assembled and operational](https://github.com/user-attachments/assets/2bedc1f4-6bcd-478d-ab8f-67a9fdcd3274)

*SmoothOperator prototype fully assembled and operational.*

Post-COVID staffing shortages in the hospitality industry have increased mobility challenges for travelers with physical impairments, particularly regarding luggage transportation. With over 61 million Americans (26% of adults) living with disabilities, there is a significant need for assistive solutions. The hospitality sector has experienced a 38% reduction in workforce since 2019, with airport ground staff decreasing by 25%, further limiting available assistance. 

SmoothOperator addresses this need through an interactive robotic assistant designed to autonomously and safely transport luggage in consumer environments. We developed a complete, full-scale, end-to-end robotic solution that seamlessly integrates multiple subsystems. This involved manufacturing custom components and assembling a sophisticated network of sensors, actuators, algorithms, and communication protocols. These systems allow SmoothOperator to navigate using either terminal destinations or direct user input via an onboard touchscreen interface. To ensure safe and reliable operation, we implemented multiple redundancy strategies and layered safety mechanisms, including dynamic obstacle avoidance that prioritizes human interaction. SmoothOperator is designed to enhance accessibility and convenience in the hospitality sector, improving the quality of life and providing greater confidence for individuals with mobility impairments when traveling.


## Current State of the Project

The SmoothOperator prototype is fully built and operational as of Spring 2025. It meets the original design metrics by carrying payloads over 50 pounds and maintaining a travel speed of at least 1.2 meters per second. The system demonstrates reliable dynamic obstacle avoidance, teleoperator remote control, real-time closed-loop feedback, and stable chassis operation. All major subsystems — mechanical, electronic, and software — have been assembled, integrated, and validated through field testing.

## System Architecture Overview

### Mechanical System

- Modular aluminum T-slot frame for stability and easy component mounting
- Two-stage lifting mechanism for cargo handling
- Shock absorbers and reinforcements for dynamic loads
- CAD files and mechanical drawings are available in the `/hardware/Version 2` directory

### Electrical System

- Dual 12V lead-acid batteries configured in parallel
- Properly-rated fusing and custom power distribution
- Separate circuits for low-voltage sensors and high-voltage actuators

### Computing and Control

- Jetson Nano for high-level processing, including navigation and UI
- Arduino Mega for low-level motor and sensor control
- Serial UART communication at 115200 baud
- Watchdog timers on both Jetson and Arduino to ensure reliability

### Software Stack

### Software Stack

The software stack is built on ROS1 (Melodic) and integrates core navigation, localization, mapping, and external web communication modules:

- `/move_base`: Central node managing global and local planning for autonomous navigation
- `/global_costmap` and `/local_costmap`: Costmaps for obstacle inflation and safe path generation
- `/global_planner` and `/local_planner`: Path planning modules within move_base
- `/recovery_behaviors`: Manages recovery actions like rotation or backing up when navigation fails
- `/cmd_vel`: Velocity command output from the local planner to control the robot base
- `/odom`: Publishes fused odometry data from wheel encoders and IMU sensors
- `/tf`: Handles sensor transforms (`odom` → `base_link`, `base_laser` → `base_link`)
- `/amcl`: Localization using particle filter based on the 2D map
- `/map`: 2D occupancy grid map generated from gmapping SLAM
- `/scan`: Real-time LiDAR scan topic for obstacle detection
- `/route_manager`: Custom API service that receives user-selected destinations (from QR codes or UI input)
- `/robot_commands`: Custom node that sends robot motion commands to a server for monitoring and logging

All nodes communicate through ROS topics, and web API endpoints are used to interface with the user interface for dynamic route management.

![System architecture overview showing subsystem connections.](https://github.com/user-attachments/assets/6b65ac43-547f-48d7-abc9-0bd1be47d619)

*System architecture overview showing subsystem connections.*


## Gotchas and Important Notes for Future Teams

- **Serial Communication Stability**: Be cautious when setting up UART links; baud rates and data flow need to be carefully matched across the Jetson, Arduino, and motor controllers to prevent dropped packets.
- **Power Management**: Watch for voltage drops under high current loads. Batteries must be properly maintained and fused.
- **Physical Assembly**: Chassis must be tightened frequently. Some aluminum members may loosen under high dynamic loads.
- **Sensor Calibration**: Ultrasonic sensors need field recalibration in noisy environments.
- **Software Deployment**: Always use SSH tunneling for Jetson Nano development to minimize transfer mistakes.
- **Navigation Debugging**: Listen to specific ROS topics to isolate bugs and validate information transfer.

## Setup and Quick Start Instructions

To get SmoothOperator operational:

1. Assemble the chassis according to the mechanical drawings (see `/hardware/Version 2/` folder).
2. Wire and fuse the electrical system as outlined in [`/hardware/README_HARDWARE.md`](./hardware/README_Hardware.md).
3. Flash the Arduino Mega with the motor and sensor firmware.
4. Set up the Jetson Nano with Ubuntu 18.04, install ROS Melodic, and deploy software per [`/software/README_SOFTWARE.md`](./software/README_Software.md).
5. Test basic manual (teleop) control before engaging autonomous navigation.
6. Calibrate ultrasonic sensors in the deployment environment.

## Future Recommendations

- Integrate visual navigation (camera-based SLAM) for enhanced indoor autonomy.
- Expand battery monitoring and fail-safe cutoff circuits.
- Implement more advanced user interfaces, including QR code boarding pass integration.
- Ruggedize sensor housings for heavy daily usage in real-world airport environments.
- Invest in a 3D LiDAR to eliminate limitations caused by obstacle height.

## Additional Documentation

- [Software README (`README_SOFTWARE.md`)](./software/README_Software.md)
- [Hardware README (`README_HARDWARE.md`)](./hardware/README_Hardware.md)
- [CAD Files and Mechanical Drawings](./hardware/Version_2)
- [Source Code](./software/)

## Team Members
- Celine Chen (ENG CE'25)
- Eric Chen (ENG CE'25)
- Jacob Chin (ENG CE'25 and ENG BME'25)
- Christian So (ENG CE'25)
- Nicholas Nguyen (ENG ME'25)

