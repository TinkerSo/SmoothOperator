#must run code with root permissions
import tkinter as tk
import serial
import time

# Set up the serial connection (replace with your Arduino's correct port)
arduino = serial.Serial(port='/dev/cu.usbmodem1101', baudrate=9600, timeout=1)
time.sleep(2)  # Give time for the serial connection to establish

def send_command(command):
    """Send a command to the Arduino."""
    arduino.write(command.encode())
    print(f"Sent command: {command}")

def key_press(event):
    """Handle key press events."""
    key = event.keysym
    if key == 'w':
        send_command('w')  # Move forward
    elif key == 's':
        send_command('s')  # Move backward
    elif key == 'a':
        send_command('a')  # Turn left
    elif key == 'd':
        send_command('d')  # Turn right

def key_release(event):
    """Handle key release events (stop motors)."""
    send_command('x')  # Stop when key is released

# Set up Tkinter window
root = tk.Tk()
root.title("Motor Control")

# Set window size (width x height)
root.geometry('400x300')

# Instructions label with larger font
label = tk.Label(root, text="Use 'w', 's', 'a', 'd' keys to control the motors", font=("Arial", 24))
label.pack(pady=20)

# Bind key press and release events to the window
root.bind('<KeyPress>', key_press)
root.bind('<KeyRelease>', key_release)

# Run the Tkinter main loop
root.mainloop()

# Close the serial connection when the window is closed
arduino.close()
