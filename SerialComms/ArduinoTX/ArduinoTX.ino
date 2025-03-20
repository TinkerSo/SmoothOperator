void setup() {
  // Start serial communication at 115200 baud rate
  Serial.begin(115200);
}

void loop() {
  // Send a message at a regular interval
  Serial.println("Hello, Jetson Nano from Arduino!");
  delay(1000); // Wait for one second
}
