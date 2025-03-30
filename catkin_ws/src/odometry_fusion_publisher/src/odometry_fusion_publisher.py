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

    # Extract the current x, y, and theta (yaw) from the Pose2D message
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    theta = data.pose.pose.position.z
    vx = data.twist.twist.linear.x
    vth = data.twist.twist.angular.z

    # Calculate the new x and y positions using the displacement and theta
    newTheta = theta - theta_old
    cycleAngle = asin(y/WHEEL_BASE)
    avgAngle = cycleAngle/2 + theta_old

    if (avgAngle > PI):
      avgAngle -= 2*PI
    elif (avgAngle < -PI):
      avgAngle += 2*PI

    x_new = x_old + x * cos(avgAngle)
    y_new = y_old + x * sin(avgAngle)
    # Update the position variables
    x_old = x_new
    y_old = y_new
    theta = cycleAngle + theta_old

    if (theta > PI):
      theta -= 2 * PI
    elif (theta < -PI):
      theta += 2 * PI

    # Log the new position
    #rospy.loginfo(f"New Position: x={x_new:.4f}, y={y_new:.4f}, theta={theta:.4f}")


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
    #quaternion = quaternion_from_euler(0.0, 0.0, theta)  # Convert from euler (roll, pitch, yaw)
    #odom_msg.pose.pose.orientation.x = quaternion[0]
    #odom_msg.pose.pose.orientation.y = quaternion[1]
    #odom_msg.pose.pose.orientation.z = quaternion[2]
    #odom_msg.pose.pose.orientation.w = quaternion[3]
    #odom_msg.pose.pose.orientation.z = theta
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

    theta_old = theta

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
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Subscribe to the /imu_encoder_pose topic
    rospy.Subscriber('/imu_encoder_pose', Odometry, pose_callback)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_pose_reader()
    except rospy.ROSInterruptException:
        pass

