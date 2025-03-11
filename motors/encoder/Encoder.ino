#include <SoftwareSerial.h>
#include "RoboClaw.h"

SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);

#define address 0x80

// Convert encoder reading to displacement in meters
double convertReadingToDistance(int32_t reading) {
  return ((2.0 * 3.14159265358979 * (0.1016 / 2.0)) / 8192.0) * reading;
}

// Convert speed reading to velocity in meters per second
double convertReadingToVelocity(int32_t reading) {
  return (((double)reading / 8192.0) * 60.0 * 2.0 * 3.14159265358979 * 0.1016) / 60.0;
}

// Send both right and left displacement & velocity data
void sendData(double right_displacement, double right_velocity, double left_displacement, double left_velocity) {
  // for debugging
  // Serial.print("Right Displacement: ");
  // Serial.print(right_displacement, 6);
  // Serial.print(" meters, Right Velocity: ");
  // Serial.print(right_velocity, 6);
  // Serial.print(" m/s | Left Displacement: ");
  // Serial.print(left_displacement, 6);
  // Serial.print(" meters, Left Velocity: ");
  // Serial.println(left_velocity, 6);

  Serial.print("Right Displacement:");
  Serial.print(right_displacement, 6);
  Serial.print(",Right Velocity:");
  Serial.print(right_velocity, 6);
  Serial.print(",Left Displacement:");
  Serial.print(left_displacement, 6);
  Serial.print(",Left Velocity:");
  Serial.println(left_velocity, 6);
}

// Display encoder and speed for both Right (M1) and Left (M2)
void displayspeed() {
  uint8_t status1, status2;
  bool valid1, valid2, valid3, valid4;
  
  // Read encoders and speeds
  int32_t enc_right = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed_right = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  int32_t enc_left = roboclaw.ReadEncM2(address, &status1, &valid3);
  int32_t speed_left = roboclaw.ReadSpeedM2(address, &status2, &valid4);

  double right_displacement = 0, right_velocity = 0;
  double left_displacement = 0, left_velocity = 0;

  if(valid1) {
    right_displacement = convertReadingToDistance(enc_right);
  }
  if(valid2) {
    right_velocity = convertReadingToVelocity(speed_right);
  }
  if(valid3) {
    left_displacement = convertReadingToDistance(enc_left);
  }
  if(valid4) {
    left_velocity = convertReadingToVelocity(speed_left);
  }

  // Send only valid data
  if ((valid1 && valid2) || (valid3 && valid4)) {
    sendData(right_displacement, right_velocity, left_displacement, left_velocity);
  }
}

// Setup function
void setup() {
  Serial.begin(115200);  
  roboclaw.begin(38400);
  delay(100);
  
  Serial.println("Starting...");
  Serial.println("Resetting encoders.");
  roboclaw.ResetEncoders(address);
}

// Main loop
void loop() {
  roboclaw.SpeedAccelDeccelPositionM1(address, 0, 12000, 0, 11000, 1);
  roboclaw.SpeedAccelDeccelPositionM1(address, 0, 12000, 0, 1000, 0);
  roboclaw.SpeedAccelDeccelPositionM1(address, 32000, 12000, 32000, 11000, 0);
  roboclaw.SpeedAccelDeccelPositionM1(address, 32000, 12000, 32000, 1000, 0);
  
  long last = millis();
  while(millis() - last < 5000) {
    displayspeed();
    delay(50);
  }
}
