#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock

def publish_clock():
    rospy.init_node('manual_clock_publisher')
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        clock_pub.publish(Clock(clock=now))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_clock()
    except rospy.ROSInterruptException:
        pass
