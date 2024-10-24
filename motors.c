/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  pinMode(2, OUTPUT);
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  leftMotorControl(-100);              // tell servo to go to position in variable 'pos'
}

int leftMotorControl(int value) {
  myservo.write(map(value, -100, 100, 1000, 2000));
}
