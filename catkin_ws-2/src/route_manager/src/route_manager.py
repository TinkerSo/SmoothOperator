#!/usr/bin/env python
import rospy
import actionlib
import json
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import requests

# Global reference to the node instance
qr_node = None
has_initialized_pose = False  # Only set hardcoded pose once

DESTINATIONS = {
    ('B', '12'): {'x': 4.053, 'y': 3.905, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.701, 'ow': 0.712},
    ('A', '0'): {'x': 0.356, 'y': -0.35, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.007, 'ow': 0.999}
}

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

def set_initial_pose(x, y, z, w):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = z
    msg.pose.pose.orientation.w = w
    msg.pose.covariance = [0.25] * 36
    pub.publish(msg)
    rospy.loginfo("Published initial pose: x=%.3f y=%.3f z=%.3f w=%.3f", x, y, z, w)

def update_initial_pose_from_amcl():
    try:
        msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5.0)
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.sleep(1)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo("Updated initial pose to current AMCL position.")
    except Exception as e:
        rospy.logwarn("Failed to update initial pose: %s", str(e))

class QRRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        global has_initialized_pose

        if self.path == '/api/QR':
            content_len = int(self.headers.getheader('content-length', 0))
            post_body = self.rfile.read(content_len)

            try:
                data = json.loads(post_body)
                gate = data.get('gate')
                terminal = data.get('terminal')

                if not gate or not terminal:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write("Missing gate or terminal")
                    return

                key = (terminal, gate)
                if key not in DESTINATIONS:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write("Unknown gate/terminal")
                    return

                rospy.loginfo("Received destination: Terminal %s, Gate %s", terminal, gate)
                pose = DESTINATIONS[key]

                # Set hardcoded initial pose only once
                if not has_initialized_pose:
                    rospy.loginfo("Setting initial pose to known starting point...")
                    set_initial_pose(0.891, -0.294, 0.002, 0.999)
                    has_initialized_pose = True

                goal = create_goal(pose)
                qr_node.client.send_goal(goal)
                qr_node.client.wait_for_result()

                result = qr_node.client.get_state()
                if result == 3:
                    rospy.loginfo("Navigation succeeded")

                    # Notify Node.js
                    try:
                        headers = { 'Content-Type': 'text/plain' }
                        requests.post("http://localhost:3000/api/ros", data="Goal Reached", headers=headers)
                        rospy.loginfo("Notified Node.js of goal completion.")
                    except Exception as e:
                        rospy.logwarn("Failed to notify Node.js: %s", str(e))

                    # Update initial pose to current location
                    # update_initial_pose_from_amcl()
                    set_initial_pose(4.053, 3.905, 0.701, 0.712)
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write("Navigation succeeded")
                else:
                    rospy.logwarn("Navigation failed")
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write("Navigation failed")

            except Exception as e:
                rospy.logerr("Exception in QR handler: %s", str(e))
                self.send_response(500)
                self.end_headers()
                self.wfile.write("Internal server error")

class RouteManagerNode(object):
    def __init__(self):
        rospy.init_node('route_manager')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("RouteManager is ready.")

        # Start HTTP server in a new thread
        server_thread = threading.Thread(target=self.start_http_server)
        server_thread.daemon = True
        server_thread.start()

    def start_http_server(self):
        server = HTTPServer(('', 8000), QRRequestHandler)
        rospy.loginfo("HTTP server started on port 8000")
        server.serve_forever()

if __name__ == '__main__':
    qr_node = RouteManagerNode()
    rospy.spin()

