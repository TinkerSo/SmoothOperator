# Project README: ESP32 and Jetson Nano UART Communication

## Overview
This project establishes a UART communication link between the ESP32 and Nvidia Jetson Nano. The ESP32 acts as a transmitter and receiver of data, interfacing with the Jetson Nano, which processes the incoming data. This setup allows for robust communication in applications ranging from simple data transfers to complex integrations such as robotics and IoT systems.

## Hardware Setup
- **ESP32 Module:** Handles data collection and transmission.
- **Nvidia Jetson Nano:** Processes received data and performs higher-level operations.
- **Connections:**
  - Connect the TX pin of the ESP32 to the RX pin of the Jetson Nano.
  - Connect the RX pin of the ESP32 to the TX pin of the Jetson Nano.
  - Ensure that both devices share a common ground.

## Software Setup
### ESP32
1. **Programming the ESP32 (`ESPJetsonCom.c`):**
   - The ESP32 is programmed with a C-based firmware file named `ESPJetsonCom.c` that facilitates UART communication.
   - Flash this firmware to the ESP32 using your preferred development environment.

### Nvidia Jetson Nano
1. **Preparing the Serial Port:**
   - Before starting your application, you need to adjust the permissions for the UART port to ensure that the Jetson Nano can communicate over it without needing superuser access:
     ```bash
     sudo chmod 666 /dev/ttyTHS1
     ```
   - This command grants read and write permissions to all users for the ttyTHS1 device, which is typically the UART port on the Jetson Nano.
2. **Installing Required Software:**
   - Should not need to do this since I already did, but in case you're wondering. Update the package list and install the required software to enable Python to communicate over serial ports:
     ```bash
     sudo apt update
     sudo apt install python3-pip
     pip3 install pyserial
     ```
3. **Running Python Script (`ESPJetson.py`):**
   - A Python script named `ESPJetson.py` is used on the Jetson Nano to receive and process the data transmitted by the ESP32.
   - Run this script after setting up the environment and making sure the ESP32 is transmitting data.
     ```bash
     python3 ESPJetson.py
     ```

## Running the Project
1. **Power on both devices.**
2. **Deploy and run `ESPJetsonCom.c` on the ESP32 and `ESPJetson.py` on the Jetson Nano.**
3. **Verify that the data transmitted by the ESP32 is accurately received and processed by the Jetson Nano.**

## Troubleshooting
- **Connection Issues:** Ensure all physical connections are secure. Double-check that TX and RX connections are correctly crossed between devices.
- **Power Issues:** Ensure that the ESP32 is powered through 'USB' and being connected to 5v on the Jetson. GND to GND.
- **Permission Errors:** If there are permission issues with accessing `/dev/ttyTHS1`, re-run the `chmod` command and verify that your user belongs to the dialout group.
