import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 115200)  # ttyTHS1 is typically the UART port on Jetson Nan

time.sleep(2)

ser.flush()

while True:
	message = "Sending Commands!\n"
	ser.write(message.encode())
	time.sleep(0.05)


