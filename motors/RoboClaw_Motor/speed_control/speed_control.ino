#include <SoftwareSerial.h>
#include "RoboClaw.h"

SoftwareSerial serial(10, 11);  
RoboClaw roboclaw(&serial, 10000);

#define address 0x80

// Velocity PID coefficients
#define Kp 0.1
#define Ki 0.2
#define Kd 0.25
#define qpps 47711  // Max Quadrature Pulses Per Second based on 350 RPM

// Robot Parameters
#define WHEEL_DIAMETER 0.1016   // 4 inches -> 0.1016 meters
#define WHEEL_CIRCUMFERENCE (3.14159265358979 * WHEEL_DIAMETER)
#define COUNTS_PER_REV 8192    // From encoder specs
#define MAX_WHEEL_SPEED_MPS ((350.0 / 60.0) * WHEEL_CIRCUMFERENCE) // Max speed in m/s

double motorSpeedMPS = 0.15; // Default speed in meters per second
char command;               // Stores user inp

// Convert speed from m/s to qpps
int mps_to_qpps(double speed_mps) {
    return (int)((speed_mps / WHEEL_CIRCUMFERENCE) * COUNTS_PER_REV);
}

// Convert encoder reading to displacement in meters
double convertReadingToDistance(int32_t reading) {
  return ((2.0 * 3.14159265358979 * (0.1016 / 2.0)) / 8192.0) * reading;
}

// Convert speed reading to velocity in meters per second
double convertReadingToVelocity(int32_t reading) {
  return (((double)reading / 8192.0) * 60.0 * 2.0 * 3.14159265358979 * 0.1016) / 60.0;
}

// Send right and left displacement & velocity data
void sendData(double right_displacement, double right_velocity, double left_displacement, double left_velocity) {
  Serial.print("Right Displacement:");
  Serial.print(right_displacement, 6);
  Serial.print(",Right Velocity:");
  Serial.print(right_velocity, 6);
  Serial.print(",Left Displacement:");
  Serial.print(left_displacement, 6);
  Serial.print(",Left Velocity:");
  Serial.println(left_velocity, 6);
}

// Display encoder and speed for both motors
void displayspeed() {
  uint8_t status1, status2;
  bool valid1, valid2, valid3, valid4;
  
  int32_t enc_right = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed_right = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  int32_t enc_left = roboclaw.ReadEncM2(address, &status1, &valid3);
  int32_t speed_left = roboclaw.ReadSpeedM2(address, &status2, &valid4);

  double right_displacement = valid1 ? convertReadingToDistance(enc_right) : 0;
  double right_velocity = valid2 ? convertReadingToVelocity(speed_right) : 0;
  double left_displacement = valid3 ? convertReadingToDistance(enc_left) : 0;
  double left_velocity = valid4 ? convertReadingToVelocity(speed_left) : 0;

  if ((valid1 && valid2) || (valid3 && valid4)) {
    sendData(right_displacement, right_velocity, left_displacement, left_velocity);
  }
}

void setup() {
  Serial.begin(115200);  
  roboclaw.begin(38400);
  delay(100);

  Serial.println("Starting...");
  Serial.println("Resetting encoders.");
  roboclaw.ResetEncoders(address);

  Serial.println("Keyboard Control Ready:");
  Serial.println("W = Forward | S = Stop | X = Backward");
  Serial.println("A = Turn Left | D = Turn Right");
  Serial.println("+ = Increase Speed | - = Decrease Speed");
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps); 
}

void loop() {
  if (Serial.available()) {
    command = Serial.read();
    
    int motorSpeedQpps = mps_to_qpps(motorSpeedMPS); // Convert m/s to qpps

    switch (command) {
      case 'w':  // Move forward
        roboclaw.SpeedM1(address, -motorSpeedQpps);
        roboclaw.SpeedM2(address, motorSpeedQpps);
        Serial.println("Moving Forward");
        break;
        
      case 'x':  // Move backward
        roboclaw.SpeedM1(address, motorSpeedQpps);
        roboclaw.SpeedM2(address, -motorSpeedQpps);
        Serial.println("Moving Backward");
        break;
        
      case 's':  // Stop
        roboclaw.SpeedM1(address, 0);
        roboclaw.SpeedM2(address, 0);
        Serial.println("Stopped");
        break;
        
      case 'a':  // Turn left (slow down left motor)
        roboclaw.SpeedM1(address, -motorSpeedQpps);
        roboclaw.SpeedM2(address, motorSpeedQpps / 2);
        Serial.println("Turning Left");
        break;
        
      case 'd':  // Turn right (slow down right motor)
        roboclaw.SpeedM1(address, -motorSpeedQpps / 2);
        roboclaw.SpeedM2(address, motorSpeedQpps);
        Serial.println("Turning Right");
        break;
        
      case '+':  // Increase speed
        motorSpeedMPS += 0.1;
        if (motorSpeedMPS > MAX_WHEEL_SPEED_MPS) motorSpeedMPS = MAX_WHEEL_SPEED_MPS;
        Serial.print("Speed increased to: ");
        Serial.print(motorSpeedMPS);
        Serial.println(" m/s");
        break;
        
      case '-':  // Decrease speed
        motorSpeedMPS -= 0.1;
        if (motorSpeedMPS < 0) motorSpeedMPS = 0;
        Serial.print("Speed decreased to: ");
        Serial.print(motorSpeedMPS);
        Serial.println(" m/s");
        break;
    }
  }

  displayspeed();  
  delay(100);
}


//#include <SoftwareSerial.h>
//#include "RoboClaw.h"
//
//SoftwareSerial serial(10, 11);  
//RoboClaw roboclaw(&serial, 10000);
//
//#define address 0x80
//
//// Velocity PID coefficients
//#define Kp 0.3
//#define Ki 0.5
//#define Kd 0.25
//#define qpps 8192
//
//int motorSpeed = 2000;  // Default speed
//char command;             // Stores user input
//
//// Convert encoder reading to displacement in meters
//double convertReadingToDistance(int32_t reading) {
//  return ((2.0 * 3.14159265358979 * (0.1016 / 2.0)) / 8192.0) * reading;
//}
//
//// Convert speed reading to velocity in meters per second
//double convertReadingToVelocity(int32_t reading) {
//  return (((double)reading / 8192.0) * 60.0 * 2.0 * 3.14159265358979 * 0.1016) / 60.0;
//}
//
//// Send right and left displacement & velocity data
//void sendData(double right_displacement, double right_velocity, double left_displacement, double left_velocity) {
//  Serial.print("Right Displacement:");
//  Serial.print(right_displacement, 6);
//  Serial.print(",Right Velocity:");
//  Serial.print(right_velocity, 6);
//  Serial.print(",Left Displacement:");
//  Serial.print(left_displacement, 6);
//  Serial.print(",Left Velocity:");
//  Serial.println(left_velocity, 6);
//}
//
//// Display encoder and speed for both motors
//void displayspeed() {
//  uint8_t status1, status2;
//  bool valid1, valid2, valid3, valid4;
//  
//  int32_t enc_right = roboclaw.ReadEncM1(address, &status1, &valid1);
//  int32_t speed_right = roboclaw.ReadSpeedM1(address, &status2, &valid2);
//  int32_t enc_left = roboclaw.ReadEncM2(address, &status1, &valid3);
//  int32_t speed_left = roboclaw.ReadSpeedM2(address, &status2, &valid4);
//
//  double right_displacement = valid1 ? convertReadingToDistance(enc_right) : 0;
//  double right_velocity = valid2 ? convertReadingToVelocity(speed_right) : 0;
//  double left_displacement = valid3 ? convertReadingToDistance(enc_left) : 0;
//  double left_velocity = valid4 ? convertReadingToVelocity(speed_left) : 0;
//
//  if ((valid1 && valid2) || (valid3 && valid4)) {
//    sendData(right_displacement, right_velocity, left_displacement, left_velocity);
//  }
//}
//
//void setup() {
//  Serial.begin(115200);  
//  roboclaw.begin(38400);
//  delay(100);
//
//  Serial.println("Starting...");
//  Serial.println("Resetting encoders.");
//  roboclaw.ResetEncoders(address);
//
//  Serial.println("Keyboard Control Ready:");
//  Serial.println("W = Forward | S = Stop | X = Backward");
//  Serial.println("A = Turn Left | D = Turn Right");
//  Serial.println("+ = Increase Speed | - = Decrease Speed");
//  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
//  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps); 
//}
//
//void loop() {
//  //roboclaw.BackwardM2(address,64); //start Motor2 backward at half speed
//  //roboclaw.SpeedM1(address, motorSpeed);
//  //roboclaw.SpeedM2(address, motorSpeed);
//  if (Serial.available()) {
//    command = Serial.read();
//    
//
//    switch (command) {
//      case 'w':  // Move forward
//        roboclaw.SpeedM1(address, -motorSpeed);
//        roboclaw.SpeedM2(address, motorSpeed);
//        Serial.println("Moving Forward");
//        break;
//        
//      case 'x':  // Move backward
//        roboclaw.SpeedM1(address, motorSpeed);
//        roboclaw.SpeedM2(address, -motorSpeed);
//        Serial.println("Moving Backward");
//        break;
//        
//      case 's':  // Stop
//        roboclaw.SpeedM1(address, 0);
//        roboclaw.SpeedM2(address, 0);
//        Serial.println("Stopped");
//        break;
//        
//      case 'a':  // Turn left (slow down left motor)
//        roboclaw.SpeedM1(address, -motorSpeed);
//        roboclaw.SpeedM2(address, motorSpeed / 2);
//        Serial.println("Turning Left");
//        break;
//        
//      case 'd':  // Turn right (slow down right motor)
//        roboclaw.SpeedM1(address, -motorSpeed / 2);
//        roboclaw.SpeedM2(address, motorSpeed);
//        Serial.println("Turning Right");
//        break;
//        
//      case '+':  // Increase speed
//        motorSpeed += 0.1;
//        if (motorSpeed > qpps) motorSpeed = qpps;
//        Serial.print("Speed increased to: ");
//        Serial.println(motorSpeed);
//        break;
//        
//      case '-':  // Decrease speed
//        motorSpeed -= 0.1;
//        if (motorSpeed < 0) motorSpeed = 0;
//        Serial.print("Speed decreased to: ");
//        Serial.println(motorSpeed);
//        break;
//    }
//  }
//
//  displayspeed();  
//  delay(100);
//}
