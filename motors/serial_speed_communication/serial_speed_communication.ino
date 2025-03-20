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

// Velocity PID coefficients
#define Kp 0.3
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

int motorSpeed = 2000;  // Default speed, Need to tune for m/s
char command;             // Stores user input

float leftMotorSpeed, rightMotorSpeed;
float wheel_width = 0.5334; // in meters

// Ultrasonic Sensor Pin (using only A0)
// #define SONAR_PIN A0
#define SONAR_PIN1 A0
int sensorValue1;
float Inch1=0.00;
float cm1=0.00;


float threshold = 20.0; // Threshold distance in cm
float cmDistance = 0.0;
float conversionFactor = 0.497 * 2.54; // approximately 1.259

// Define LED Variable color
int led_color[3];



// Set LEDs
void setLEDs(uint32_t color) {
  strip.fill(color);
  strip.show();
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

// Function to convert linear velocity (V) and angular velocity (omega) 
// into left and right wheel speeds for a tank drive system
void tankDrive(float V, float omega, float W, float &leftSpeed, float &rightSpeed) {
    leftSpeed = V - (omega * W / 2);
    rightSpeed = V + (omega * W / 2);
}

void setup() {
  Serial.begin(115200);
  roboclaw.begin(38400);
  delay(100);
  
  while (!Serial) {}  
  // Initialize NeoPixels
  strip.begin();
  strip.show();
  strip.setBrightness(80);
  setLEDs(strip.Color(255, 0, 0)); 
  pinMode(SONAR_PIN1,INPUT);
  float Inch1=0.00;
  float cm1=0.00;

  
  Serial.println("Starting...");
  Serial.println("Resetting encoders.");
  roboclaw.ResetEncoders(address);

  Serial.println("Keyboard Control Ready:");
  Serial.println("W = Forward | S = Stop | X = Backward");
  Serial.println("A = Turn Left | D = Turn Right");
  Serial.println("+ = Increase Speed | - = Decrease Speed");
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps); 

  // Robot initially stopped
  led_color[0] = 255; // Red
  led_color[1] = 0;
  led_color[2] = 0;
  setLEDs(strip.Color(led_color[0], led_color[1], led_color[2]));


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
    String input = Serial.readStringUntil('\n'); // Read the entire line
    int Vx, Vy, Vtheta;

    // Parse three integer values from the input
    //V-x V-y V-theta
    sscanf(input.c_str(), "%d %d %d", &Vx, &Vy, &Vtheta);

    if(Vx == 0 or Vy == 0 or Vtheta == 0){ // STOPPED
      led_color[0] = 255; // Red
      led_color[1] = 0;
      led_color[2] = 0;
    }
    else{ // MOVING
      led_color[0] = 0;
      led_color[1] = 255; //green
      led_color[2] = 0;
    }

    // Set LED lights to specified color
    setLEDs(strip.Color(led_color[0], led_color[1], led_color[2]));

    tankDrive(Vx, Vtheta, wheel_width, leftMotorSpeed, rightMotorSpeed);
    


    // Assign motor speed to motors 
    // M1 is left motor (inverted, so, negative is forwards). But, tankdrive will handle this negative
    // M2 is right motor (normal, so, positive is forwards)
    roboclaw.SpeedM1(address, leftMotorSpeed);
    roboclaw.SpeedM2(address, rightMotorSpeed);
    
    //char command = Serial.read();  // Read single character

//    switch (command) {
//      case 'w':
//        roboclaw.ForwardBackwardM1(address, 94); 
//        roboclaw.ForwardBackwardM2(address, 94); 
//        setLEDs(strip.Color(0, 255, 0));  // Green for movement
//        break;
//      case 's':
//        roboclaw.ForwardBackwardM1(address, 33); 
//        roboclaw.ForwardBackwardM2(address, 33); 
//        setLEDs(strip.Color(0, 255, 0));  
//        break;
//      case 'a':
//        roboclaw.ForwardBackwardM1(address, 94); 
//        roboclaw.ForwardBackwardM2(address, 33); 
//        setLEDs(strip.Color(0, 255, 0));  
//        break;
//      case 'd':
//        roboclaw.ForwardBackwardM1(address, 33); 
//        roboclaw.ForwardBackwardM2(address, 94); 
//        setLEDs(strip.Color(0, 255, 0));  
//        break;
//      case 'x':
//        roboclaw.ForwardBackwardM1(address, 64);
//        roboclaw.ForwardBackwardM2(address, 64);
//        setLEDs(strip.Color(255, 0, 0));
//        break;
//      default:
//        // Ignore unknown keys
//        break;
//    }

    // Flush serial buffer
    while (Serial.available()) Serial.read();
  }

  delay(50);
}
