import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', 115200)  # ttyTHS1 is typically the UART port on Jetson Nano
time.sleep(2)
while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        # print(line)
        #line = ser.readline().decode('ascii', errors='ignore').rstrip()
        print(f"Raw Data: {line}")  # Print raw output
    #time.sleep(0.1)
