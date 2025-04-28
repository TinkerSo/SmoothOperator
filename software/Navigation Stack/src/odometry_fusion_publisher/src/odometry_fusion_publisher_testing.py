#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from math import sin, cos, pi, atan2
import tf
from tf2_ros import TransformBroadcaster

# Global variables for position tracking
x_old = 0.0
y_old = 0.0
theta_old = 0.0
WHEEL_BASE = 0.5334  # Center of left tire to center of right tire
PI = pi

# Scaling factor for delta_theta calculation - adjust this to calibrate rotation
# Start with 0.25 which is a 1/4 scale factor to fix the 
ROTATION_SCALE = 0.167

br = TransformBroadcaster()

# Callback function for IMU pose data
def pose_callback(data):
    global x_old, y_old, theta_old

    # Extract the current data from the Odometry message
    # x is the average of both wheels' displacements
    x_avg = data.pose.pose.position.x
    # y is the difference of right and left wheel displacements
    wheel_diff = data.pose.pose.position.y
    vx = data.twist.twist.linear.x
    vth = data.twist.twist.angular.z

    # Calculate change in orientation based on the difference in wheel distances
    # Apply scaling factor to correct the rotation measurement
    delta_theta = (wheel_diff / WHEEL_BASE) * ROTATION_SCALE
    
    # # Debug logging to verify rotation values
    # rospy.logdebug(f"wheel_diff: {wheel_diff:.4f}, delta_theta: {delta_theta:.4f}")
    
    # Calculate new orientation
    theta_new = theta_old + delta_theta
    
    # Normalize theta to [-pi, pi]
    if theta_new > PI:
        theta_new -= 2 * PI
    elif theta_new < -PI:
        theta_new += 2 * PI
    
    # Calculate the new position based on arc motion
    if abs(delta_theta) > 0.001:  # Robot is turning
        # For arc motion, calculate the radius of the turn
        # Since we've scaled delta_theta, we need to adjust the radius calculation
        radius = x_avg / delta_theta if delta_theta != 0 else 0
        
        # For an arc motion, the position change in the robot's local frame is:
        delta_x_local = radius * sin(delta_theta)
        delta_y_local = radius * (1 - cos(delta_theta))
        
        # Transform the local displacement to the global frame
        # Using the orientation at the start of the movement
        delta_x_global = delta_x_local * cos(theta_old) - delta_y_local * sin(theta_old)
        delta_y_global = delta_x_local * sin(theta_old) + delta_y_local * cos(theta_old)
    else:
        # For straight line motion, the displacement is simply in the direction of theta_old
        delta_x_global = x_avg * cos(theta_old)
        delta_y_global = x_avg * sin(theta_old)
    
    # Update position
    x_new = x_old + delta_x_global
    y_new = y_old + delta_y_global
    
    # Debug output for monitoring position updates
    
    
    # Update old values for next iteration
    x_old = x_new
    y_old = y_new
    theta_old = theta_new

    # Create and publish the Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # Set position
    odom_msg.pose.pose.position.x = x_new
    odom_msg.pose.pose.position.y = y_new
    odom_msg.pose.pose.position.z = 0.0  # Assuming 2D motion, z = 

    # Set orientation
    q = quaternion_from_euler(0, 0, theta_new)
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

    # Enable debug output
    rospy.loginfo("Odometry node started with rotation scale: %.2f", ROTATION_SCALE)
    
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_pose_reader()
    except rospy.ROSInterruptException:
        pass