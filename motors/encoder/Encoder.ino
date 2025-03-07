//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

const char ENCODER_START_BYTE = 0xFF; // Message start byte for encoder data
const char ENCODER_END_BYTE = 0xEE; // Message end byte for encoder data

// Send both displacement and velocity readings through serial
void sendData(int32_t displacement, int32_t velocity) {
  Serial.write(ENCODER_START_BYTE);
  // Sending displacement one byte at a time starting from MSB
  Serial.write((displacement >> 24) & 0xFF);
  Serial.write((displacement >> 16) & 0xFF);
  Serial.write((displacement >> 8) & 0xFF);
  Serial.write(displacement & 0xFF);
  // Sending velocity one byte at a time starting from MSB
  Serial.write((velocity >> 24) & 0xFF);
  Serial.write((velocity >> 16) & 0xFF);
  Serial.write((velocity >> 8) & 0xFF);
  Serial.write(velocity & 0xFF);
  // Sending end byte
  Serial.write(ENCODER_END_BYTE);
}

// Convert encoder reading to displacement in meters
double convertReadingToDistance(int32_t reading) {
  float numerator = (2 * 3.14 * (.1016 / 2)) / 8192;
  return numerator * reading;
}

// Convert speed reading to velocity in meters per second
double convertReadingToVelocity(int32_t reading) {
  return ((reading / 8192) * 60 * 2 * 3.14 * .1016) / 60;
}
//Display Encoder and Speed for Motor 1
void displayspeed(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  double displacement, velocity;
  
  // Convert enc1 reading to revolution
  if(valid1){
    Serial.print("Encoder1:");
    Serial.print(enc1,DEC);
    Serial.print(" ");
    displacement = convertReadingToDistance(enc1);
    Serial.print(displacement);
    Serial.print(" Meters moved.");
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  if(valid2){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
    velocity = convertReadingToVelocity(speed1);
    Serial.print(velocity);
    Serial.print(" m/s");
    Serial.print(" ");
  }
  
  if (valid1 && valid2) {
    // Both readings are valid
    sendData(displacement, velocity);
  }
  else if (valid1) {
    // Only displacement is valid
    sendData(displacement, 0);
  }
  else {
    // Only velocity is valid
    sendData(0, velocity);
  }
  Serial.println();
}

//This is the first function arduino runs on reset/power up
void setup() {
  //Open Serial and roboclaw at 38400bps
  Serial.begin(115200);
  roboclaw.begin(38400);
  
  Serial.println("Starting...");
  Serial.println("Resetting encoders.");
  roboclaw.ResetEncoders(address);
}

void loop() {
  roboclaw.SpeedAccelDeccelPositionM1(address,0,12000,0,11000,1);
  roboclaw.SpeedAccelDeccelPositionM1(address,0,12000,0,1000,0);
  roboclaw.SpeedAccelDeccelPositionM1(address,32000,12000,32000,11000,0);
  roboclaw.SpeedAccelDeccelPositionM1(address,32000,12000,32000,1000,0);
  long last = millis();
  while(millis()-last<5000){
    displayspeed();
    delay(50);
  }
}
