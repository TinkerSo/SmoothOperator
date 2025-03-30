#!/usr/bin/env python
import rospy
import actionlib
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# List of goals
GOALS = [
    {'x': 4.254, 'y': 3.460, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.705, 'ow': 0.708}
   # {'x': 4.508, 'y': 0.108, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.003, 'ow': 0.999},
   # {'x': 4.508, 'y': 0.108, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.711, 'ow': 0.704},
   # {'x': 4.452, 'y': 4.037, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.710, 'ow':0.704},


    #{'x':0.356, 'y': -0.035, 'z': 0.000, 'ox': 0.000, 'oy':0.000, 'oz': -0.007, 'ow': 0.999}
]

def amcl_ready():
    rospy.loginfo("Waiting for AMCL to start")
    msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=10)
    rospy.loginfo("AMCL active")
    return msg is not None

def set_initial_pose(x, y, z, w):
    """Publishes an initial pose estimate to /initialpose."""
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # Ensure the publisher is set up

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"

    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.position.z = 0.0  # 2D navigation, z should be 0

    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = z
    initial_pose.pose.pose.orientation.w = w

    # Set covariance (default values)
    initial_pose.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                                    0, 0.25, 0, 0, 0, 0,
                                    0, 0, 0.25, 0, 0, 0,
                                    0, 0, 0, 0.25, 0, 0,
                                    0, 0, 0, 0, 0.25, 0,
                                    0, 0, 0, 0, 0, 0.06853]  # Yaw uncertainty

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

def send_goals_sequentially(goals):
    rospy.init_node('multi_goal_sender')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base")

    # Set initial pose
    if amcl_ready():
        set_initial_pose(0.356, -0.035, -0.007, 0.999)
       # set_initial_pose(4.264, 3.460, 0.705, 0.708)

    for idx, pose in enumerate(goals):
        goal = create_goal(pose)
        rospy.loginfo("Sending goal")
        client.send_goal(goal)

        # Wait for result
        client.wait_for_result()
        result = client.get_state()

        if result == 3:  # SUCCEEDED
            rospy.loginfo("Goal reached!")
        else:
            rospy.logwarn("Goal failed with state, skipping to next.")

    rospy.loginfo("All goals processed. Done.")

if __name__ == '__main__':
    try:
        send_goals_sequentially(GOALS)
    except rospy.ROSInterruptException:
        pass

