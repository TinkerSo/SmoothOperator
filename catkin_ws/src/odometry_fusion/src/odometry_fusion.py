#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
import serial
from std_msgs.msg import Header

# Constants
SERIAL_PORT = "/dev/ttyACM0"
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

# Initialize Serial Communication
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.setDTR(False)
    ser.flushInput()
    #rospy.loginfo(f"Serial port {SERIAL_PORT} opened successfully.")
except serial.SerialException:
    #rospy.logerr(f"Failed to open serial port {SERIAL_PORT}. Check connection.")
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

            #rospy.loginfo(f"Absolute Displacement: {absolute_displacement:.4f} m | Yaw: {yaw:.4f} rad")

            return absolute_displacement, average_velocity
            #return 0.0, 0.0
        except Exception as e:
            #rospy.logwarn(f"Malformed Serial Data: {serial_data} | Error: {e}")
            #print "Error"
            return 0.0, 0.0
    return 0.0, 0.0

# Callback function for IMU data
def imu_callback(data):
    global yaw  # Access global yaw variable
    global vtheta

    vtheta = data.angular_velocity.z

    # Extract quaternion from the IMU message
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    )

    #print quaternion
    # Create tf2 listener to transform quaternion to Euler angles
    listener = tf2_ros.TransformListener(tf2_ros.Buffer())
    try:
        euler = tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
        yaw = euler[2]  # Rotation around Z-axis (yaw)
    except Exception as e:
        rospy.logwarn("Error transforming quaternion to Euler angles")

# Main function
def imu_encoder_reader():
    global yaw, absolute_displacement, odom

    # Initialize the node
    rospy.init_node('imu_encoder_reader', anonymous=True)

    # Publisher for yaw and absolute displacement
    pose_pub = rospy.Publisher('/imu_encoder_pose', Odometry, queue_size=10)

    # Subscribe to IMU data topic
    #rospy.Subscriber('/imu/data', Imu, imu_callback)

    rate = rospy.Rate(100)  # 10 Hz loop rate
    while not rospy.is_shutdown():
        # Read encoder displacement
        displacement, velocity = read_encoder_displacement()

        odom = Odometry()

        odom.header = Header()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        #print displacement
        if displacement is not None:
            # Create and populate pose data
            odom.pose.pose = Pose()
            #odom.pose.pose.position = Point(x=displacement, y=displacement_difference, z=yaw)
            odom.pose.pose.position = Point(x=displacement, y=displacement_difference, z=0.0)
            #rospy.loginfo(f"Published: x={pose_msg.x:.4f}, theta={pose_msg.theta:.4f}")

        if velocity is not None:
            odom.twist.twist = Twist()
            odom.twist.twist.linear = Vector3(x=velocity, y=0.0, z=0.0) # Linear velocity
            #odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=vtheta)
            odom.twist.twist.angular= Vector3(x=0.0, y=0.0, z=0.0)

        pose_pub.publish(odom)

        rate.sleep()

if __name__ == '__main__':
    try:
        imu_encoder_reader()
    except rospy.ROSInterruptException:
        pass

