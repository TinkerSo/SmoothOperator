import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyTHS1', 115200)  # ttyTHS1 is typically the UART port on Jetson Nano

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
    time.sleep(1)
