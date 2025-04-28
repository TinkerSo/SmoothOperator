#include "RoboClaw.h"
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// NeoPixel Configuration
#define LED_PIN    6   
#define LED_COUNT  300  
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// RoboClaw Configuration
SoftwareSerial softwareSerial(10, 11); 
#define address 0x80
#define baudrate 38400
RoboClaw roboclaw(&softwareSerial, 10000);

// Ultrasonic Sensor Pin (using only A0)
// #define SONAR_PIN A0
#define SONAR_PIN1 A0
int sensorValue1;
float Inch1=0.00;
float cm1=0.00;


float threshold = 20.0; // Threshold distance in cm
float cmDistance = 0.0;
float conversionFactor = 0.497 * 2.54; // approximately 1.259

void setup() {
  Serial.begin(115200);
  
  while (!Serial) {}  
  // Initialize NeoPixels
  strip.begin();
  strip.show();
  strip.setBrightness(80);
  setLEDs(strip.Color(255, 0, 0)); 
  pinMode(SONAR_PIN1,INPUT);
  float Inch1=0.00;
  float cm1=0.00;

  // Initialize RoboClaw
  softwareSerial.begin(baudrate);
  roboclaw.begin(baudrate);
}

void loop() {
  // Read ultrasonic sensor (A0) and convert to cm
  sensorValue1=analogRead(SONAR_PIN1);
  Inch1= (sensorValue1*0.497);
  cm1=Inch1*2.54;
  // Serial.println(cm1);

  // Check for obstacles
  if (false) {
  // if (cmDistance < threshold) {
    strip.fill(strip.Color(255, 0, 0));
    strip.show();
    // Stop MOtors
    roboclaw.ForwardBackwardM1(address, 64);
    roboclaw.ForwardBackwardM2(address, 64);
    setLEDs(strip.Color(255, 0, 0));
    delay(50);
    return;
  }

  // Check for keyboard input
  if (Serial.available()) {
    char command = Serial.read();  // Read single character

    switch (command) {
      case 'w':
        roboclaw.ForwardBackwardM1(address, 94); 
        roboclaw.ForwardBackwardM2(address, 94); 
        setLEDs(strip.Color(0, 255, 0));  // Green for movement
        break;
      case 's':
        roboclaw.ForwardBackwardM1(address, 33); 
        roboclaw.ForwardBackwardM2(address, 33); 
        setLEDs(strip.Color(0, 255, 0));  
        break;
      case 'a':
        roboclaw.ForwardBackwardM1(address, 94); 
        roboclaw.ForwardBackwardM2(address, 33); 
        setLEDs(strip.Color(0, 255, 0));  
        break;
      case 'd':
        roboclaw.ForwardBackwardM1(address, 33); 
        roboclaw.ForwardBackwardM2(address, 94); 
        setLEDs(strip.Color(0, 255, 0));  
        break;
      case 'x':
        roboclaw.ForwardBackwardM1(address, 64);
        roboclaw.ForwardBackwardM2(address, 64);
        setLEDs(strip.Color(255, 0, 0));
        break;
      default:
        // Ignore unknown keys
        break;
    }

    // Flush serial buffer
    while (Serial.available()) Serial.read();
  }

  delay(50);
}


// Set LEDs
void setLEDs(uint32_t color) {
  strip.fill(color);
  strip.show();
}
