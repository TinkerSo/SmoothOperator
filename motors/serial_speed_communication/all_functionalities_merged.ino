#include "RoboClaw.h"
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// Define L298N Motor Driver Pins
#define IN1 7
#define IN2 6
#define ENA 5

// Define Bumper Switch pins
#define front_bump 22
#define back_bump 24
#define left_bump 26
#define right_bump 28

// NeoPixel Configuration
#define LED_PIN    8   
#define LED_COUNT  300  
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// RoboClaw Configuration
SoftwareSerial softwareSerial(10, 11); 
#define address 0x80
#define baudrate 38400
RoboClaw roboclaw(&softwareSerial, 10000);

// Velocity PID coefficients
#define Kp 0.3
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

#define WHEEL_DIAMETER 0.1016
#define WHEEL_CIRCUMFERENCE (3.14159265358979 * WHEEL_DIAMETER)
#define COUNTS_PER_REV 8192

int motorSpeed = 2000;
char command;

float leftMotorSpeed, rightMotorSpeed;
float wheel_width = 0.5334;

#define SONAR_PIN1 A0
int sensorValue1;
float Inch1=0.00;
float cm1=0.00;

float threshold = 20.0;
float conversionFactor = 0.497 * 2.54;

int led_color[3];

void setLEDs(uint32_t color) {
  strip.fill(color);
  strip.show();
}

double convertReadingToDistance(int32_t reading) {
  return ((2.0 * 3.14159265358979 * (0.1016 / 2.0)) / 8192.0) * reading;
}

double convertReadingToVelocity(int32_t reading) {
  return (((double)reading / 8192.0) * 60.0 * 2.0 * 3.14159265358979 * 0.1016) / 60.0;
}

void sendData(double right_displacement, double left_displacement, double right_velocity, double left_velocity) {
  Serial1.print("Right Displacement:");
  Serial1.print(right_displacement, 6);
  Serial1.print(",Left Displacement:");
  Serial1.print(left_displacement, 6);
  Serial1.print(",Right Velocity:");
  Serial1.print(right_velocity, 6);
  Serial1.print(",Left Velocity:");
  Serial1.println(left_velocity, 6);
}

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
    sendData(-right_displacement, left_displacement, -right_velocity, left_velocity);
  }
}

int mps_to_qpps(double speed_mps) {
  return (int)((speed_mps / WHEEL_CIRCUMFERENCE) * COUNTS_PER_REV);
}

void tankDrive(float V, float omega, float W, float &leftSpeed, float &rightSpeed) {
    // leftSpeed = V - (omega * W / 2);
    // rightSpeed = V + (omega * W / 2);

    leftSpeed = V + (omega * W / 2);
    rightSpeed = V - (omega * W / 2);
}

void liftUp() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
}

void liftDown() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
}

void liftStop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
}

void setLiftSpeed(float speed) {
    analogWrite(ENA, speed*255);
}

bool hasThreeDecimalPlaces(String val) {
  int dotIndex = val.indexOf('.');
  if (dotIndex == -1) return false;
  int digitsAfterDecimal = val.length() - dotIndex - 1;
  return digitsAfterDecimal == 3;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  roboclaw.begin(38400);
  delay(100);

  while (!Serial) {}
  strip.begin();
  strip.show();
  strip.setBrightness(80);
  setLEDs(strip.Color(255, 0, 0)); 
  pinMode(SONAR_PIN1,INPUT);

  Serial.println("Starting...");
  roboclaw.ResetEncoders(address);

  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps); 

  led_color[0] = 255; led_color[1] = 0; led_color[2] = 0;
  setLEDs(strip.Color(led_color[0], led_color[1], led_color[2]));

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(left_bump, INPUT);
  pinMode(right_bump, INPUT);
  pinMode(front_bump, INPUT);
  pinMode(back_bump, INPUT);
}

void loop() {
  sensorValue1=analogRead(SONAR_PIN1);
  Inch1= (sensorValue1*0.497);
  cm1=Inch1*2.54;

  displayspeed();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.print("Raw input: ");
    Serial.println(input);

    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);
    int thirdSpace = input.indexOf(' ', secondSpace + 1);

    if (firstSpace == -1 || secondSpace == -1 || thirdSpace == -1) {
      Serial.println("Error: Not enough spaces in input.");
      roboclaw.SpeedM1(address, 0);
      roboclaw.SpeedM2(address, 0);
      liftStop();
    } else {
      String Vx_str = input.substring(0, firstSpace);
      String Vy_str = input.substring(firstSpace + 1, secondSpace);
      String Vtheta_str = input.substring(secondSpace + 1, thirdSpace);
      String lift_str = input.substring(thirdSpace + 1);

      if (!hasThreeDecimalPlaces(Vx_str) || !hasThreeDecimalPlaces(Vy_str) || 
          !hasThreeDecimalPlaces(Vtheta_str) || !hasThreeDecimalPlaces(lift_str)) {
        Serial.println("Error: Values must have exactly 3 decimal places.");
        roboclaw.SpeedM1(address, 0);
        roboclaw.SpeedM2(address, 0);
        liftStop();
      } else {
        float Vx = Vx_str.toFloat();
        float Vy = Vy_str.toFloat();
        float Vtheta = Vtheta_str.toFloat();
        float lift_action = lift_str.toFloat();

        Serial.print("Parsed Vx: "); Serial.println(Vx, 3);
        Serial.print("Parsed Vy: "); Serial.println(Vy, 3);
        Serial.print("Parsed Vtheta: "); Serial.println(Vtheta, 3);
        Serial.print("Parsed lift: "); Serial.println(lift_action, 3);

        if (Vx == 0 && Vy == 0 && Vtheta == 0) {
          setLEDs(strip.Color(255, 0, 0));
        } else {
          setLEDs(strip.Color(0, 255, 0));
        }

        if (lift_action < 0) {
          setLiftSpeed(-lift_action);
          liftDown();
        } else if (lift_action > 0) {
          setLiftSpeed(lift_action);
          liftUp();
        } else {
          liftStop();
        }

        tankDrive(Vx, Vtheta, wheel_width, leftMotorSpeed, rightMotorSpeed);

        int leftQpps = constrain(mps_to_qpps(leftMotorSpeed), -qpps, qpps);
        int rightQpps = constrain(mps_to_qpps(rightMotorSpeed), -qpps, qpps);

        roboclaw.SpeedM1(address, -leftQpps);
        roboclaw.SpeedM2(address, rightQpps);
      }
    }

    while (Serial.available()) Serial.read();
  }

  delay(50);
}
