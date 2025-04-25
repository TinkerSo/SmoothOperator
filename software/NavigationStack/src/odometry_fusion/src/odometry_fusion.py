#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import smbus
import time
import math
import numpy as np
import serial

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from std_msgs.msg import Header

# Constants
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Global variables
yaw = 0.0
prev_right_displacement = 0.0
prev_left_displacement = 0.0
absolute_displacement = 0.0
right_velocity = 0.0
left_velocity = 0.0
vtheta = 0.0
odom = ""
displacement_difference = 0.0

class MPU6050:
    # MPU6050 Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    FIFO_EN = 0x23
    INT_ENABLE = 0x38
    INT_STATUS = 0x3A
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    TEMP_OUT_H = 0x41
    WHO_AM_I = 0x75
    
    # Scaling factors
    ACCEL_SCALE_2G = 16384.0  # LSB per g
    ACCEL_SCALE_4G = 8192.0   # LSB per g
    ACCEL_SCALE_8G = 4096.0   # LSB per g
    ACCEL_SCALE_16G = 2048.0  # LSB per g
    
    GYRO_SCALE_250 = 131.0    # LSB per degree per second
    GYRO_SCALE_500 = 65.5     # LSB per degree per second
    GYRO_SCALE_1000 = 32.8    # LSB per degree per second
    GYRO_SCALE_2000 = 16.4    # LSB per degree per second
    
    def __init__(self, address=0x68, bus_num=1):
        self.address = address
        self.bus = smbus.SMBus(bus_num)  # Use smbus instead of smbus2
        
        # Set up the device
        self.setup()
        
        # Initialize calibration offsets
        self.accel_offsets = np.array([0.0, 0.0, 0.0])
        self.gyro_offsets = np.array([0.0, 0.0, 0.0])
        
        # Current orientation in radians
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Filter parameters
        self.alpha = 0.98  # Complementary filter coefficient
        
        # Scaling - default to +/- 2g and +/- 250 degrees per second
        self.accel_scale = self.ACCEL_SCALE_2G
        self.gyro_scale = self.GYRO_SCALE_250
    
    def setup(self):
        """Initialize the MPU6050."""
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x01)
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 0x00)
        self.bus.write_byte_data(self.address, self.CONFIG, 0x03)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
    
    def read_raw_data(self, addr):
        """Read 2 bytes from register address and return signed 16-bit value."""
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        
        value = (high << 8) | low
        
        if value > 32767:
            value = value - 65536
        
        return value
    
    def read_accel(self):
        """Read accelerometer data in g."""
        ax = self.read_raw_data(self.ACCEL_XOUT_H) / self.accel_scale - self.accel_offsets[0]
        ay = self.read_raw_data(self.ACCEL_XOUT_H + 2) / self.accel_scale - self.accel_offsets[1]
        az = self.read_raw_data(self.ACCEL_XOUT_H + 4) / self.accel_scale - self.accel_offsets[2]
        
        return np.array([ax, ay, az])
    
    def read_gyro(self):
        """Read gyroscope data in degrees per second."""
        gx = self.read_raw_data(self.GYRO_XOUT_H) / self.gyro_scale - self.gyro_offsets[0]
        gy = self.read_raw_data(self.GYRO_XOUT_H + 2) / self.gyro_scale - self.gyro_offsets[1]
        gz = self.read_raw_data(self.GYRO_XOUT_H + 4) / self.gyro_scale - self.gyro_offsets[2]
        
        return np.array([gx, gy, gz])
    
    def read_temp(self):
        """Read temperature in degrees Celsius."""
        temp_raw = self.read_raw_data(self.TEMP_OUT_H)
        temp_celsius = (temp_raw / 340.0) + 36.53
        return temp_celsius
    
    def calibrate(self, samples=100, delay=0.01):
    # Calibrate the IMU by finding the zero offset.
        print("Calibrating MPU6050 - keep the sensor still...")
        
        accel_samples = []
        gyro_samples = []
        
        for _ in range(samples):
            try:
                accel = self.read_accel()
                gyro = self.read_gyro()
                
                accel_samples.append(accel)
                gyro_samples.append(gyro)
                
                time.sleep(delay)
            except Exception as e:
                print("Error during calibration")
                continue
        
        if accel_samples and gyro_samples:
            # Calculate mean offsets
            accel_mean = np.mean(accel_samples, axis=0)
            # For accelerometer: when still, x and y should be 0, z should be 1g
            self.accel_offsets = np.array([accel_mean[0], accel_mean[1], accel_mean[2] - 1.0])
            
            # Gyroscope should read 0 when still
            self.gyro_offsets = np.mean(gyro_samples, axis=0)
            
            print("Calibration complete!")
            print("Accel offsets")
            print("Gyro offsets")
        else:
            print("Calibration failed - no valid samples collected")
            
    def update_orientation(self, dt):
        """Update orientation using a complementary filter."""
        try:
            # Read sensor data
            accel = self.read_accel()
            gyro = self.read_gyro()
            
            # Convert gyro rates to radians per second
            gyro_rad = np.radians(gyro)
            
            # Integrate gyro rates to get angles
            self.roll += gyro_rad[0] * dt
            self.pitch += gyro_rad[1] * dt
            self.yaw += gyro_rad[2] * dt
            
            # Calculate accel angles
            accel_roll = math.atan2(accel[1], accel[2])
            accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
            
            # Complementary filter
            # Use a lower alpha value to make accelerometer influence stronger
            alpha = 0.8  # Reduced from 0.98
            
            # Only use accelerometer data if it's reliable (magnitude close to 1g)
            accel_magnitude = np.linalg.norm(accel)
            if 0.8 < accel_magnitude < 1.2:  # Within 20% of expected gravity
                self.roll = alpha * self.roll + (1.0 - alpha) * accel_roll
                self.pitch = alpha * self.pitch + (1.0 - alpha) * accel_pitch
            
            # Keep roll and pitch in range [-pi, pi]
            if self.roll > math.pi:
                self.roll -= 2 * math.pi
            elif self.roll < -math.pi:
                self.roll += 2 * math.pi
                
            if self.pitch > math.pi:
                self.pitch -= 2 * math.pi
            elif self.pitch < -math.pi:
                self.pitch += 2 * math.pi
                
            if self.yaw > math.pi:
                self.yaw -= 2 * math.pi
            elif self.yaw < -math.pi:
                self.yaw += 2 * math.pi
            
            return self.roll, self.pitch, self.yaw
            
        except Exception as e:
            return self.roll, self.pitch, self.yaw

    def get_orientation_degrees(self):
        """Get orientation in degrees."""
        return (
            math.degrees(self.roll),
            math.degrees(self.pitch),
            math.degrees(self.yaw)
        )

    def close(self):
        """Close the SMBus connection (only needed for smbus, not smbus2)."""
        self.bus.close()

# Initialize Serial Communication
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.setDTR(False)
    ser.flushInput()
except serial.SerialException:
    exit()

def read_encoder_displacement():
    """ Reads encoder displacement from Arduino and calculates absolute displacement. """
    global prev_right_displacement, prev_left_displacement, absolute_displacement, right_velocity, left_velocity, displacement_difference

    if ser.in_waiting:
        serial_data = ser.readline().decode().strip()
        #print serial_data
        try:
            parts = serial_data.split(',')
            # print parts
            right_displacement = float(parts[0].split(':')[1])
            left_displacement = float(parts[1].split(':')[1])
            right_velocity = float(parts[2].split(':')[1])
            left_velocity = float(parts[3].split(':')[1])

            # Compute displacement change
            delta_right = right_displacement - prev_right_displacement
            delta_left = left_displacement - prev_left_displacement

            # Update previous values
            prev_right_displacement = right_displacement
            prev_left_displacement = left_displacement

            absolute_displacement = (delta_right + delta_left)/2.0

            # Calculate average velocity
            average_velocity = (right_velocity + left_velocity) / 2.0

            displacement_difference = delta_right - delta_left

            return absolute_displacement, average_velocity
        except Exception as e:
            return 0.0, 0.0
    return 0.0, 0.0

# Main function
def imu_encoder_reader():
    global yaw, absolute_displacement, odom

    # Initialize the node
    rospy.init_node('imu_encoder_reader', anonymous=True)

    # Publisher for yaw and absolute displacement
    pose_pub = rospy.Publisher('/imu_encoder_pose', Odometry, queue_size=100)

    # Initialize time tracking
    last_time = time.time()

    rate = rospy.Rate(100) # Sampling at 100hz
    while not rospy.is_shutdown():
        # <--------------------------IMU------------------------------->
        # Calculate time elapsed since last reading
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Update orientation
        roll, pitch, yaw = imu.update_orientation(dt)
        
        # Convert to degrees for display
        roll_deg, pitch_deg, yaw_deg = imu.get_orientation_degrees()
        
        # Read and display acceleration and gyro data
        accel = imu.read_accel()
        gyro = imu.read_gyro()
        temp = imu.read_temp()

        # <--------------------------Encoder------------------------------->
        # Read encoder displacement
        displacement, velocity = read_encoder_displacement()

        odom = Odometry()

        odom.header = Header()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        if displacement is not None:
            # Create and populate encoder and imu data
            odom.pose.pose = Pose()
            yaw_radians = (yaw_deg * math.pi) / 180
            odom.pose.pose.position = Point(x=displacement, y=displacement_difference, z=yaw_radians)

        if velocity is not None:
            odom.twist.twist = Twist()
            odom.twist.twist.linear = Vector3(x=velocity, y=0.0, z=0.0) # Linear velocity
            odom.twist.twist.angular= Vector3(x=0.0, y=0.0, z=0.0)

        # Publish odometry data
        pose_pub.publish(odom)

        rate.sleep()

if __name__ == '__main__':
    try:
        # Create IMU object with the correct address 0x68
        imu = MPU6050(address=0x68, bus_num=1)
        
        # Calibrate the sensor
        imu.calibrate()
        
        imu_encoder_reader()
    except rospy.ROSInterruptException:
        pass

