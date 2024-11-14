import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyTHS1', 115200)  # ttyTHS1 is the UART port on Jetson Nano

try:
    while True:
        ser.write(b"Hello ESP32\n")  # Send a string to the ESP32
        time.sleep(1)  # Wait for a second before sending the next string
except KeyboardInterrupt:
    ser.close()  # Close the serial port when the script is interrupted
