import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyTHS1', 115200)  # ttyTHS1 is the UART port on Jetson Nano

try:
    while True:
        ser.write("w".encode('utf-8'))  # Send a string to the ESP32
        time.sleep(2)  # Wait for a second before sending the next string
except KeyboardInterrupt:
    ser.close()  # Close the serial port when the script is interrupted
