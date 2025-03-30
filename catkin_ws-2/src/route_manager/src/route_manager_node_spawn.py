#!/usr/bin/env python
import rospy
import actionlib
import tf
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Map terminal + gate to destination pose
DESTINATIONS = {
    ('B', '12'): {'x': 4.264, 'y': 3.460, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.705, 'ow': 0.708}
}

def amcl_ready():
    rospy.loginfo("Waiting for AMCL to start")
    try:
        msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=10)
        rospy.loginfo("AMCL active")
        return msg is not None
    except rospy.ROSException:
        rospy.logwarn("Timed out waiting for AMCL")
        return False

def set_initial_pose(x, y, z, w):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = z
    initial_pose.pose.pose.orientation.w = w
    initial_pose.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                                    0, 0.25, 0, 0, 0, 0,
                                    0, 0, 0.25, 0, 0, 0,
                                    0, 0, 0, 0.25, 0, 0,
                                    0, 0, 0, 0, 0.25, 0,
                                    0, 0, 0, 0, 0, 0.0685]

    rospy.loginfo("Setting initial pose")
    pub.publish(initial_pose)

def create_goal(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose['x']
    goal.target_pose.pose.position.y = pose['y']
    goal.target_pose.pose.position.z = pose['z']
    goal.target_pose.pose.orientation.x = pose['ox']
    goal.target_pose.pose.orientation.y = pose['oy']
    goal.target_pose.pose.orientation.z = pose['oz']
    goal.target_pose.pose.orientation.w = pose['ow']
    return goal

def send_goal(destination_pose):
    rospy.init_node('qr_navigator', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base")

    if amcl_ready():
        set_initial_pose(0.356, -0.035, -0.007, 0.999)  # hardcoded for now

    goal = create_goal(destination_pose)
    rospy.loginfo("Sending navigation goal")
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_state()
    if result == 3:
        rospy.loginfo("Navigation succeeded")
    else:
        rospy.logwarn("Navigation failed, state = %d" % result)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: navigate_qr_node.py [GATE] [TERMINAL]")
        sys.exit(1)

    gate = sys.argv[1]
    terminal = sys.argv[2]

    key = (terminal, gate)
    if key not in DESTINATIONS:
        rospy.logerr("Unknown destination: Terminal %s, Gate %s" % (terminal, gate))
        sys.exit(1)

    try:
        send_goal(DESTINATIONS[key])
    except rospy.ROSInterruptException:
        pass

