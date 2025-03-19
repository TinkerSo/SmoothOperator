#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7); // RX, TX (for communication with Arduino 1)

void setup() {
  Serial.begin(9600);  // Serial Monitor for debugging
  mySerial.begin(9600); // Communication with Arduino 1
}

void loop() {
  if (mySerial.available() > 0) {
    String receivedData = mySerial.readStringUntil('\n');
    Serial.print("Distance Received: ");
    Serial.print(receivedData);
    Serial.println(" cm");
  }
}
