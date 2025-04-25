#!/usr/bin/env python
import rospy
import actionlib
import json
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading

# Global reference to the node instance
qr_node = None

DESTINATIONS = {
    ('B', '12'): {'x': 4.166, 'y': 3.576, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.700, 'ow': 0.714}
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

class QRRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
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

                set_initial_pose(0.574, -0.491, 0.002, 0.999)

                goal = create_goal(pose)
                qr_node.client.send_goal(goal)
                qr_node.client.wait_for_result()

                result = qr_node.client.get_state()
                if result == 3:
                    rospy.loginfo("Navigation succeeded")
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write("Navigation succeeded")
                else:
                    rospy.logwarn("Navigation failed")
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write("Navigation failed")
            except Exception as e:
                rospy.logerr("Exception in HTTP handler: %s", str(e))
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
        server = HTTPServer(('localhost', 8000), QRRequestHandler)
        rospy.loginfo("HTTP server started on port 8000")
        server.serve_forever()

if __name__ == '__main__':
    qr_node = RouteManagerNode()
    rospy.spin()

