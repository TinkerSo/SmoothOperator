#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from math import sin, cos, asin
import tf
from tf2_ros import TransformBroadcaster

# Global variables for position tracking
x_old = 0.0
y_old = 0.0
theta_old = 0.0
WHEEL_BASE = .5334 # Center of left tire to center of right tire
PI = 3.141592

br = TransformBroadcaster()

# Callback function for IMU pose data
def pose_callback(data):
    global x_old, y_old, theta_old

    # Extract the current x, y, and yaw from the Pose2D message
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    theta = data.pose.pose.position.z
    vx = data.twist.twist.linear.x
    vth = data.twist.twist.angular.z

    # Calculate the new x and y positions using the displacement and theta
    x_new = x_old + x * cos(theta)
    y_new = y_old + x * sin(theta)

    # Update the position variables
    x_old = x_new
    y_old = y_new

    # Create and publish the Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"  # Assuming the base frame is 'odom'
    odom_msg.child_frame_id = "base_link"

    # Set position
    odom_msg.pose.pose.position.x = x_new
    odom_msg.pose.pose.position.y = y_new
    odom_msg.pose.pose.position.z = 0.0  # Assuming 2D motion, z = 0

    # Set orientation (quaternion from euler)
    q = quaternion_from_euler(0, 0, theta)
    odom_quat = Quaternion()
    odom_quat.x = q[0]
    odom_quat.y = q[1]
    odom_quat.z = q[2]
    odom_quat.w = q[3]

    odom_msg.pose.pose.orientation = odom_quat
    odom_msg.twist.twist.linear.x = vx
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.linear.z = 0.0
    odom_msg.twist.twist.angular.z = vth

    # Publish the odometry message
    odom_pub.publish(odom_msg)

    # Broadcast the transform
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"

    # Set translation (position)
    transform.transform.translation.x = x_new
    transform.transform.translation.y = y_new
    transform.transform.translation.z = 0.0

    # Set rotation (quaternion)
    transform.transform.rotation = odom_quat

    # Send the transform
    br.sendTransform(transform)

# Main function
def imu_pose_reader():
    # Initialize the node
    rospy.init_node('imu_pose_reader', anonymous=True)

    # Publisher for Odometry messages
    global odom_pub
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)

    # Subscribe to the /imu_encoder_pose topic
    rospy.Subscriber('/imu_encoder_pose', Odometry, pose_callback)

    rate = rospy.Rate(50) # Sampling at 50hz

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_pose_reader()
    except rospy.ROSInterruptException:
        pass

