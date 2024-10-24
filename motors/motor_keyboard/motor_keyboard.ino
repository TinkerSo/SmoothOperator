#include <Servo.h>

Servo right_motor;
Servo left_motor;

void setup() {
  Serial.begin(9600);
  left_motor.attach(2);
  right_motor.attach(3);
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    control_motors(input);
  }
}

void control_motors(char command) {
  switch (command) {
    case 'w':  // Forward
      move_motor(left_motor, 100);
      move_motor(right_motor, -100);
      break;
    case 's':  // Backward
      move_motor(left_motor, -100);
      move_motor(right_motor, 100);
      break;
    case 'a':  // Left
      move_motor(left_motor, -50);
      move_motor(right_motor, -100);
      break;
    case 'd':  // Right
      move_motor(left_motor, 100);
      move_motor(right_motor, 50);
      break;
    case 'x':  // Stop
      move_motor(left_motor, 0);
      move_motor(right_motor, 0);
      break;
  }
}

void move_motor(Servo motor, int value) {
  motor.write(map(value, -100, 100, 1000, 2000));
}
