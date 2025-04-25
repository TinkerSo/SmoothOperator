#include <SoftwareSerial.h>

#define TRIG_PIN 9  // Ultrasonic sensor TRIG pin
#define ECHO_PIN 10 // Ultrasonic sensor ECHO pin

SoftwareSerial mySerial(8, 7); // RX, TX (for communication with Arduino 2)

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  mySerial.begin(9600);  // Communication with Arduino 2
}

void loop() {
  long duration;
  int distance;

  // Send pulse to trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo duration
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // Convert duration to distance in cm

  // Send distance value to Arduino 2
  mySerial.println(distance);
  
  delay(1000); // Send data every second
}
