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
