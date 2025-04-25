#!/usr/bin/env python3

import rospy
import serial
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# Constants
WHEEL_BASE = 0.54  # Distance between left and right wheels (meters)

# Global Variables
prev_right_distance = 0.0
prev_left_distance = 0.0
x, y, th = 0.0, 0.0, 0.0  # Position & Orientation

# Initialize Serial Connection
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    rospy.loginfo("Serial port /dev/ttyACM0 opened successfully.")
except serial.SerialException:
    rospy.logerr("Failed to open serial port! Check connection.")
    exit()

def publish_imu_data(vx, vth):
    """Publishes velocity as /imu/data (even though it's from wheel encoders)."""
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "imu_link"

    # Assign linear and angular velocity
    imu_msg.linear_acceleration.x = vx  # Forward velocity (m/s)
    imu_msg.angular_velocity.z = vth    # Rotational velocity (rad/s)

    imu_pub.publish(imu_msg)
    rospy.loginfo("Publishing /imu/data -> Linear: %.4f m/s | Angular: %.4f rad/s", vx, vth)

def publish_odometry(vx, vth, dt):
    """Computes and publishes odometry to /odom."""
    global x, y, th, prev_right_distance, prev_left_distance

    # Compute position update
    delta_s = vx * dt
    delta_th = vth * dt

    x += delta_s * math.cos(th + delta_th / 2.0)
    y += delta_s * math.sin(th + delta_th / 2.0)
    th += delta_th
    th = math.atan2(math.sin(th), math.cos(th))  # Normalize theta

    # Create Odometry Message
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.orientation.z = math.sin(th / 2.0)
    odom_msg.pose.pose.orientation.w = math.cos(th / 2.0)

    odom_msg.twist.twist.linear.x = vx
    odom_msg.twist.twist.angular.z = vth

    odom_pub.publish(odom_msg)
    rospy.loginfo("Publishing /odom -> x: %.4f, y: %.4f, θ: %.4f", x, y, th)

def main():
    global prev_right_distance, prev_left_distance

    rospy.init_node("odometry_publisher")
    global imu_pub, odom_pub
    imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz loop
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        # Read Serial Data from Encoders
        if ser.in_waiting:
            serial_data = ser.readline().decode().strip()
            rospy.loginfo("Raw Serial Data: %s", serial_data)

            try:
                parts = serial_data.split(',')
                right_distance = float(parts[0].split(':')[1])
                left_distance = float(parts[1].split(':')[1])

                # Compute per-loop displacement (ΔD)
                delta_s_right = right_distance - prev_right_distance
                delta_s_left = left_distance - prev_left_distance

                # Update previous displacement values
                prev_right_distance = right_distance
                prev_left_distance = left_distance

                # Compute forward velocity and angular velocity
                vx = (delta_s_right + delta_s_left) / (2.0 * dt)
                vth = (delta_s_right - delta_s_left) / (WHEEL_BASE * dt)

                # Publish IMU Data (Wheel-based velocity)
                publish_imu_data(vx, vth)

                # Publish Odometry
                publish_odometry(vx, vth, dt)

            except Exception as e:
                rospy.logwarn("Malformed Serial Data: %s | Error: %s", serial_data, str(e))

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


