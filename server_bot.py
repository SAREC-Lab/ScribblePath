#!/usr/bin/env python2.7

#Server imports
# from http.server import BaseHTTPRequestHandler, HTTPServer
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
# import socketserver
import json

PORT = 8080

#turtlebot imports

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

x = 0.0
y = 0.0 
theta = 0.0

# Callback function
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def turtle(waypoints): 
    # Initialize mode
    rospy.init_node("speed_controller")

    # Subscribe to the odom topic to get information about the current position and velocity
    # of the robot
    sub = rospy.Subscriber("/odom", Odometry, newOdom)

    # Publish linear and angular velocities to cmd_vel topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    speed = Twist()

    r = rospy.Rate(4)

    # Establish target coordinates
    goal = Point()
    current_goal=0
    goal.x=waypoints[current_goal][0]
    goal.y=waypoints[current_goal][1]
    # Strategy is to first turn to face the target coordinates
    # and then move towards them
    while not rospy.is_shutdown() and current_goal < len(waypoints):

        # Compute difference between current position and target position
        inc_x = goal.x -x
        inc_y = goal.y -y
        angle_to_goal = atan2(inc_y, inc_x)
        
        difference_angle = abs(angle_to_goal - theta)
        msg2 = ("x: {},  y: {}, velocity: {}, angle: {}".format(x,y,speed.linear.x,theta))
        rospy.loginfo(msg2)
        dist = sqrt(inc_x**2 + inc_y**2)
        # Check whether to turn or move.
        angle_change=angle_to_goal-theta
        if dist<.5:
            speed.angular.z=0
            speed.linear.x=0
            current_goal+=1
            if current_goal < len(waypoints):
                goal.x=waypoints[current_goal][0]
                goal.y=waypoints[current_goal][1]

        elif abs(angle_change) > 0.1:
            #speed.linear.x = 0.0
            if angle_change > 0:
                speed.angular.z = 0.13
            else:
                speed.angular.z = -0.13
        else:
            rospy.loginfo("Ready to move")
            speed.linear.x = dist*0.05+.025
            speed.angular.z = 0.0
        
        print("curr: {} length_waypoint: {}".format(current_goal, len(waypoints)))
        pub.publish(speed)
        r.sleep()


# Server Class
class Handler(BaseHTTPRequestHandler):
    last_coords = [0,0]

    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        pass

    def do_POST(self):
        content_length = int(self.headers['Content-Length']) # Size of Data
        data = self.rfile.read(content_length).decode('utf-8') # Data
        coords = json.loads(data)
        print("Before Loop {}".format(coords))
        for c in coords:
            c[0] += self.last_coords[0]
            c[1] += self.last_coords[1]
        
        print("Coords {}, Last: {}".format(coords,self.last_coords))
        turtle(coords)
        self.last_coords = coords[-1]


        self._set_response()
        self.wfile.write("Recieved".encode('utf-8'))

def run(server_class=HTTPServer, handler_class=Handler, port=PORT):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print('Starting server...\n')
    try:
        print('PORT: ' + str(PORT))
        print('URL: localhost:' + str(PORT))
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    print('Stopping server...\n')

if __name__ == '__main__':
    run()
