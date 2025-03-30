#!/usr/bin/env python3

import rospy
import smbus2
import struct
from sensor_msgs.msg import Imu

# Constants for the ICM-20948
ICM20948_I2C_ADDR = 0x69
ACCEL_XOUT_H = 0x2D
GYRO_XOUT_H = 0x33

bus = smbus2.SMBus(1)

def read_registers(addr, num_bytes):
    """Read multiple bytes from a contiguous block of registers."""
    readValue = bus.read_i2c_block_data(ICM20948_I2C_ADDR, addr, num_bytes)
    return readValue

def read_imu_data():
    """Reads IMU acceleration and gyroscope data."""
    accel_data = read_registers(ACCEL_XOUT_H, 6)
    gyro_data = read_registers(GYRO_XOUT_H, 6)

    accel = struct.unpack('>hhh', bytes(accel_data))
    gyro = struct.unpack('>hhh', bytes(gyro_data))

    accel_g = [x * (2.0 / 32768.0) for x in accel]  # Convert to g's
    gyro_rps = [x * (3.14159 / 180.0 * 250.0 / 32768.0) for x in gyro]  # Convert to rad/s

    return accel_g, gyro_rps

def imu_publisher():
    rospy.init_node('imu_publisher', anonymous=True)
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        accel, gyro = read_imu_data()

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass


