#include <RoboClaw.h>
#include <SoftwareSerial.h>

// SoftwareSerial configuration
SoftwareSerial softwareSerial(10, 11); // RX on pin 10, TX on pin 11

// RoboClaw configuration
#define address 0x80  // Default address of the RoboClaw as set on the device
#define baudrate 38400 // Set this to match the baud rate of your RoboClaw

// Ultrasonic sensor variables
int SonarPin = A0; // Example pin for an ultrasonic sensor
float threshold = 20.0; // Threshold distance in cm
int sensorValue = 0;
float distance = 0.0;

// Create RoboClaw instance using SoftwareSerial
RoboClaw roboclaw(&softwareSerial, 10000); // Second parameter is the timeout

void setup() {
  // Initialize serial communication with the computer
  Serial.begin(115200);
  Serial.println("Serial communication started on the main serial port.");

  // Initialize SoftwareSerial communication
  softwareSerial.begin(baudrate);
  roboclaw.begin(baudrate);

  // Set up ultrasonic sensor
  pinMode(SonarPin, INPUT);
}

void loop() {
  // Check if there is incoming data
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String received = Serial.readStringUntil('\n');

    // Print the received data to the serial monitor
    Serial.print("Received: ");
    Serial.println(received.trim());

    // Parse command and control motors
    controlMotors(received.trim());
  }
}

void controlMotors(String command) {
  // Read the ultrasonic sensor
  sensorValue = analogRead(SonarPin);
  distance = sensorValue * 0.497 * 2.54; // Convert sensor value to distance in cm

  if (distance <= threshold) {
    // If an obstacle is detected within the threshold, stop the motors
    roboclaw.ForwardBackwardM1(address, 64); // Stop Motor 1
    roboclaw.ForwardBackwardM2(address, 64); // Stop Motor 2
    Serial.println("Obstacle detected. Motors stopping.");
    return; // Early return to prevent executing movement commands
  }

  // Execute motor commands only if no obstacle is detected
  if (command == "w") {
    roboclaw.ForwardBackwardM1(address, 127);
    roboclaw.ForwardBackwardM2(address, 127);
    Serial.println("Motors moving forward");
  } else if (command == "s") {
    roboclaw.ForwardBackwardM1(address, 0);
    roboclaw.ForwardBackwardM2(address, 0);
    Serial.println("Motors moving backward");
  } else if (command == "a") {
    roboclaw.ForwardBackwardM1(address, 127);
    roboclaw.ForwardBackwardM2(address, 0);
    Serial.println("Turning left");
  } else if (command == "d") {
    roboclaw.ForwardBackwardM1(address, 0);
    roboclaw.ForwardBackwardM2(address, 127);
    Serial.println("Turning right");
  } else if (command == "x") {
    roboclaw.ForwardBackwardM1(address, 64);
    roboclaw.ForwardBackwardM2(address, 64);
    Serial.println("Motors braking");
  }
}
