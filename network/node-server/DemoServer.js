#include "RoboClaw.h"
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// L298N Motor Driver Pins
#define IN1 9
#define IN2 8
#define ENA 7

// Bumper Switch Pins
#define FRONT_BUMP 50
#define BACK_BUMP 44
#define LEFT_BUMP 38
#define RIGHT_BUMP 32

// Ultrasonic Sensor Pins
#define TRIG_PIN 5
#define ECHO_PIN 6

// NeoPixel Configuration
#define LED_PIN 11
#define LED_COUNT 300
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// RoboClaw Configuration
SoftwareSerial softwareSerial(12, 13);
#define ROBOCLAW_ADDRESS 0x80
#define BAUDRATE 38400
RoboClaw roboclaw(&softwareSerial, 10000);

// PID Coefficients
#define Kp 0.1
#define Ki 0.1
#define Kd 0.25
#define QPPS 47711

// Wheel & Encoder Constants
#define WHEEL_DIAMETER 0.1016
#define WHEEL_CIRCUMFERENCE (3.14159265358979 * WHEEL_DIAMETER)
#define COUNTS_PER_REV 8192

// Sonar Sensor Pin
#define SONAR_PIN1 A0

// Motion Variables
int motorSpeed = 2000;
float leftMotorSpeed, rightMotorSpeed;
const float WHEEL_WIDTH = 0.5334;
const float DISTANCE_THRESHOLD = 40.0;

// LED State
int led_color[3] = {255, 0, 0};

// Function to Set LEDs
void setLEDs(uint32_t color) {
  strip.fill(color);
  strip.show();
}

// Convert Encoder Reading to Distance
double convertReadingToDistance(int32_t reading) {
  return ((2.0 * 3.14159265358979 * (WHEEL_DIAMETER / 2.0)) / COUNTS_PER_REV) * reading;
}

// Convert Encoder Reading to Velocity
double convertReadingToVelocity(int32_t reading) {
  return ((double)reading / COUNTS_PER_REV) * (3.14159265358979 * WHEEL_DIAMETER);
}

// Send Data Over Serial
void sendData(double right_displacement, double left_displacement, double right_velocity, double left_velocity) {
  Serial1.print("Right Displacement: "); Serial1.print(right_displacement, 6);
  Serial1.print(", Left Displacement: "); Serial1.print(left_displacement, 6);
  Serial1.print(", Right Velocity: "); Serial1.print(right_velocity, 6);
  Serial1.print(", Left Velocity: "); Serial1.println(left_velocity, 6);
}

// Display Speed from Encoders
void displayspeed() {
  uint8_t status;
  bool valid1, valid2, valid3, valid4;

  int32_t enc_right = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS, &status, &valid1);
  int32_t speed_right = roboclaw.ReadSpeedM1(ROBOCLAW_ADDRESS, &status, &valid2);
  int32_t enc_left = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS, &status, &valid3);
  int32_t speed_left = roboclaw.ReadSpeedM2(ROBOCLAW_ADDRESS, &status, &valid4);

  double right_displacement = valid1 ? convertReadingToDistance(enc_right) : 0;
  double right_velocity = valid2 ? convertReadingToVelocity(speed_right) : 0;
  double left_displacement = valid3 ? convertReadingToDistance(enc_left) : 0;
  double left_velocity = valid4 ? convertReadingToVelocity(speed_left) : 0;

  if ((valid1 && valid2) || (valid3 && valid4)) {
    sendData(-right_displacement, left_displacement, -right_velocity, left_velocity);
  }
}

// Convert Meters Per Second to QPPS
int mps_to_qpps(double speed_mps) {
  return (int)((speed_mps / WHEEL_CIRCUMFERENCE) * COUNTS_PER_REV);
}

// Tank Drive Calculation
void tankDrive(float V, float omega, float W, float &leftSpeed, float &rightSpeed) {
  leftSpeed = V + (omega * W / 2);
  rightSpeed = V - (omega * W / 2);
}

// Lift Control Functions
void liftUp() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
void liftDown() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
void liftStop() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
void setLiftSpeed(float speed) { analogWrite(ENA, speed * 255); }

// Check if String has Exactly 3 Decimal Places
bool hasThreeDecimalPlaces(String val) {
  int dotIndex = val.indexOf('.');
  return (dotIndex != -1 && (val.length() - dotIndex - 1) == 3);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  roboclaw.begin(BAUDRATE);

  strip.begin();
  strip.show();
  strip.setBrightness(80);
  setLEDs(strip.Color(255, 0, 0));

  pinMode(SONAR_PIN1, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(FRONT_BUMP, INPUT);
  pinMode(BACK_BUMP, INPUT);
  pinMode(LEFT_BUMP, INPUT);
  pinMode(RIGHT_BUMP, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Starting...");
  roboclaw.ResetEncoders(ROBOCLAW_ADDRESS);
  roboclaw.SetM1VelocityPID(ROBOCLAW_ADDRESS, Kd, Kp, Ki, QPPS);
  roboclaw.SetM2VelocityPID(ROBOCLAW_ADDRESS, Kd, Kp, Ki, QPPS);
}

void loop() {
  displayspeed();
  

  // Read Sonar Sensor
//  digitalWrite(TRIG_PIN, LOW);
//  delayMicroseconds(2);
//  digitalWrite(TRIG_PIN, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG_PIN, LOW);
//  int distance = pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
//  Serial.println(distance);
  //Serial1.println(distance);

  // Check for Obstacles
//  bool obstacleDetected = (distance < DISTANCE_THRESHOLD || 
//    digitalRead(LEFT_BUMP) || digitalRead(RIGHT_BUMP) || 
//    digitalRead(FRONT_BUMP) || digitalRead(BACK_BUMP));
    bool obstacleDetected = (digitalRead(LEFT_BUMP) || digitalRead(RIGHT_BUMP) || 
    digitalRead(FRONT_BUMP) || digitalRead(BACK_BUMP));

  if (obstacleDetected) {
    roboclaw.SpeedM1(ROBOCLAW_ADDRESS, 0);
    roboclaw.SpeedM2(ROBOCLAW_ADDRESS, 0);
    setLEDs(strip.Color(255, 0, 0));
  } 
  else if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.print("Raw input: "); //debugging line
    Serial.println(input);//debugging line

    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);
    int thirdSpace = input.indexOf(' ', secondSpace + 1);

    if (firstSpace == -1 || secondSpace == -1 || thirdSpace == -1) {
      Serial.println("Error: Invalid input format.");
      liftStop();
    } else {
      String Vx_str = input.substring(0, firstSpace);
      String Vy_str = input.substring(firstSpace + 1, secondSpace);
      String Vtheta_str = input.substring(secondSpace + 1, thirdSpace);
      String lift_str = input.substring(thirdSpace + 1);

      if (!hasThreeDecimalPlaces(Vx_str) || !hasThreeDecimalPlaces(Vtheta_str) || !hasThreeDecimalPlaces(lift_str)) {
        Serial.println("Error: Values must have exactly 3 decimal places.");
        liftStop();
        roboclaw.SpeedM1(ROBOCLAW_ADDRESS, 0);
        roboclaw.SpeedM2(ROBOCLAW_ADDRESS, 0);
      } else {
        float Vx = Vx_str.toFloat(), Vy = Vy_str.toFloat(), Vtheta = Vtheta_str.toFloat(), lift_action = lift_str.toFloat();
        if (lift_action != 0) setLiftSpeed(abs(lift_action)), lift_action > 0 ? liftUp() : liftDown();
        else liftStop();

        tankDrive(Vx, Vtheta, WHEEL_WIDTH, leftMotorSpeed, rightMotorSpeed);
        roboclaw.SpeedM1(ROBOCLAW_ADDRESS, -mps_to_qpps(leftMotorSpeed));
        roboclaw.SpeedM2(ROBOCLAW_ADDRESS, mps_to_qpps(rightMotorSpeed));
        if (Vx != 0.000 || Vy != 0.000 || Vtheta != 0.000)
        {
          setLEDs(strip.Color(0, 255, 0));
        } else {
          setLEDs(strip.Color(255, 0, 0));

        }
      }
    }
  }

  delay(20);
}
