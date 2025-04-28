# SmoothOperator
ECE Senior Design 2024-2025  
Team 30

# SmoothOperator - Engineering Addendum

## Table of Contents
- [1. Project Overview](#1-project-overview)
- [2. Current State of the Project](#2-current-state-of-the-project)
- [3. System Architecture Overview](#3-system-architecture-overview)
- [4. Gotchas and Important Notes for Future Teams](#4-gotchas-and-important-notes-for-future-teams)
- [5. Setup and Quick Start Instructions](#5-setup-and-quick-start-instructions)
- [6. Future Recommendations](#6-future-recommendations)
- [7. Additional Documentation](#7-additional-documentation)

---

## Project Overview
SmoothOperator is a full-scale, semi-autonomous robotic platform designed to transport luggage and heavy cargo in consumer environments, particularly airports. It integrates dynamic obstacle avoidance, teleoperator control, a lifting mechanism, an onboard user interface, an user mobile application, and a stable, modular chassis to provide intuitive and safe assistance to travelers.

## Current State of the Project
The SmoothOperator prototype is fully built and operational as of Spring 2025. It meets the original design metrics by carrying payloads over 50 pounds and maintaining a travel speed of at least 1.2 meters per second. The system demonstrates reliable dynamic obstacle avoidance, teleoperator remote control, real-time closed-loop feedback, and stable chassis operation. All major subsystems—mechanical, electronic, and software—have been assembled, integrated, and validated through field testing.

## System Architecture Overview
The robot consists of:
- A modular aluminum frame with a lifting mechanism
- A lead-acid powered electrical system managed via custom circuitry
- An onboard Jetson Nano for high-level decision making
- An Arduino Mega handling low-level motor and sensor interfaces
- A robust software stack for obstacle detection, teleop control, and safety switching

![Screenshot 2025-04-27 at 10 12 01 PM](https://github.com/user-attachments/assets/6b65ac43-547f-48d7-abc9-0bd1be47d619)

## Gotchas and Important Notes for Future Teams
- **Serial Communication Stability**: Be cautious when setting up UART links; baud rates and data flow need to be carefully matched across the Jetson, Arduino, and motor controllers to prevent dropped packets.
- **Power Management**: Watch for voltage drops under high current loads. Batteries must be properly maintained and fused.
- **Physical Assembly**: Chassis must be tightened frequently. Some aluminum members may loosen under high dynamic loads.
- **Sensor Calibration**: Ultrasonic sensors need field recalibration in noisy environments.
- **Software Deployment**: Always use SSH tunneling for Jetson Nano development; direct development minimizes transfer mistakes.
- **Navigation Depugging**: Listen to specific topics to isolate bugs and validate transfer of information

## Setup and Quick Start Instructions
To get SmoothOperator operational:
1. Assemble chassis according to mechanical drawings (see `/cad/` folder).
2. Set up electrical wiring according to `/hardware/README_HARDWARE.md`.
3. Flash Arduino Mega with control firmware.
4. Deploy and launch Jetson Nano software stack following `/software/README_SOFTWARE.md`.
5. Test basic motion using teleoperator control before engaging autonomous modes.

## Future Recommendations
- Integrate visual navigation (camera-based SLAM) for enhanced indoor autonomy.
- Expand battery monitoring and fail-safe cutoff circuits.
- Implement more advanced user interfaces, including QR code boarding pass integration.
- Ruggedize sensor housings for heavy daily usage in real-world airport environments.

## Additional Documentation
- [Software Report (README_SOFTWARE.md)](./software/README_Software.md)
- [Hardware Report (README_HARDWARE.md)](./hardware/README_Hardware.md)
- [CAD Files and Mechanical Drawings](./cad/)
- [Source Code](./software/)
