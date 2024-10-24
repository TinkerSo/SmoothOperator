#include <Servo.h>

Servo right_motor; //Forward is negative values 
Servo left_motor; //Forward is positive values


void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  left_motor.attach(2); 
  right_motor.attach(3);  
}

void loop() {
  move_motor(left_motor, 100);              
  move_motor(right_motor, -100);
}

int move_motor(Servo motor, int value) {
  motor.write(map(value, -100, 100, 1000, 2000)); //Maps servo values to proper PWM signal values
}
