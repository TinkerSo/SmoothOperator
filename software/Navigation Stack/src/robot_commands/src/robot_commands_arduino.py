#!/usr/bin/env python2

import rospy
import requests
import serial
import time
from geometry_msgs.msg import Twist

class RobotCommands:

    def __init__(self):
        self.ser = serial.Serial(
                 port='/dev/ttyACM0',
                 baudrate=115200,
                 timeout=1
        )

        rospy.init_node('robot_commands', anonymous=True)

        # Subscribe to cmd_vel topic
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # Set up rate
        self.rate = rospy.Rate(10)  # 100 Hz

        rospy.loginfo("Robot Commands Node Started")
        rospy.spin()

    def cmd_vel_callback(self, msg):
        """
        Callback function to process received velocity commands
        """
        vx = msg.linear.x  # Forward velocity
        vy = msg.linear.y  # Sideways velocity
        vtheta = msg.angular.z  # Rotational velocity

        rospy.loginfo("Received cmd_vel: vx=%.3f, vy=%.3f, vtheta=%.3f" % (vx, vy, vtheta))

        # Convert to motor commands
        self.send_motor_commands(vx, vy, vtheta)

    '''
    def send_motor_commands(self, vx, vy, vtheta):
        """
        Sends motor commands to the Jetson Nano server
        """
        # Formatting data to three decimal places
        data = "ROS, {:.3f}, {:.3f}, {:.3f}".format(vx, vy, vtheta)

        try:
            response = requests.post(self.JETSON_SERVER_URL, data=data, headers={"Content-Type": "text/plain"})
            if response.status_code == 200:
                rospy.loginfo("Sent to Jetson: " + data)
            else:
                rospy.logwarn("Failed to send data: " + response.text)
        except requests.exceptions.RequestException as e:
            rospy.logerr("Error sending data: " + str(e))

    '''
    def send_motor_commands(self, vx, vy, vtheta):
	if self.ser.is_open:
          data = "{:.3f} {:.3f} {:.3f} 0.000\n".format(vx, vy, vtheta)
	  self.ser.write(data.encode()) # Send motor commands
          rospy.loginfo(data)
        else:
	   print("Serial port is not open.")

if __name__ == "__main__":
    try:
        RobotCommands()
    except rospy.ROSInterruptException:
        pass

