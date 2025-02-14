void setup() {
  // Start serial communication at 115200 baud rate
  Serial.begin(115200);
}

void loop() {
  // Check if there is incoming data
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String received = Serial.readStringUntil('\n');

    // Print the received data to the serial monitor
    Serial.print("Received: ");
    Serial.println(received);
  }
}
