#include <Servo.h>

Servo right_motor;  // Forward is negative values
Servo left_motor;   // Forward is positive values

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  left_motor.attach(2); 
  right_motor.attach(3);
  
  // Starting message
  Serial.println("Enter 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right, and 'x' to stop.");
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();  // Read keyboard input from serial
    control_motors(input);       // Call function to control motors based on input
  }
}

void control_motors(char command) {
  switch (command) {
    case 'w':  // Move forward
      move_motor(left_motor, 100);
      move_motor(right_motor, -100);
      break;

    case 's':  // Move backward
      move_motor(left_motor, -100);
      move_motor(right_motor, 100);
      break;

    case 'a':  // Turn left
      move_motor(left_motor, -50);
      move_motor(right_motor, -100);
      break;

    case 'd':  // Turn right
      move_motor(left_motor, 100);
      move_motor(right_motor, 50);
      break;

    case 'x':  // Stop
      move_motor(left_motor, 0);
      move_motor(right_motor, 0);
      break;

    default:
      Serial.println("Invalid command. Use 'w', 's', 'a', 'd', or 'x'.");
      break;
  }
}

int move_motor(Servo motor, int value) {
  motor.writeMicroseconds(map(value, -100, 100, 1000, 2000));  // Maps values to PWM signals
}
