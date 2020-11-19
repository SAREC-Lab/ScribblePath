#!/usr/bin/env python2.7

#Server imports
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import json
import threading
import thread

PORT = 8080

#turtlebot imports
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, pi
from sensor_msgs.msg import LaserScan

x = 0.0
y = 0.0 
theta = 0.0
last_coords = [0,0]
turtle_thread = None
stop = False

# Callback function
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def obstacle_detection_callback(dt):
    global stop
    global pub

    print '-------------------------------------------'
    print 'Range data at 0 deg:   {}'.format(dt.ranges[0])
    print 'Range data at 15 deg:  {}'.format(dt.ranges[15])
    print 'Range data at 345 deg: {}'.format(dt.ranges[345])
    print '-------------------------------------------'

    # Thresholds
    thr1 = 0.3
    thr2 = 0.3

    # Checks if obstacles in front and 15 deg left and right
    if not (dt.ranges[0] > thr1 and dt.ranges[15] > thr2 and dt.ranges[345] > thr2):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        stop = True
        print("OBSTACLE!!!")

    pub.publish(speed)

# Initialize mode
rospy.init_node("speed_controller")
# rospy.init_node('obstacle_avoidance_node')

# Publish linear and angular velocities to cmd_vel topic
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

# Subscribe to the odom topic to get information about the current position and velocity
# of the robot
sub = rospy.Subscriber("/odom", Odometry, newOdom)
#sub1 = rospy.Subscriber("/scan", LaserScan, obstacle_detection_callback)

speed = Twist()

# Initialize mode
rospy.init_node("speed_controller")

# Subscribe to the odom topic to get information about the current position and velocity
# of the robot
sub = rospy.Subscriber("/odom", Odometry, newOdom)

# Publish linear and angular velocities to cmd_vel topic
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

rate = rospy.Rate(10)

def turtle(waypoints):
    global stop
    
    # Establish target coordinates
    goal = Point()
    current_goal=0
    goal.x=waypoints[current_goal][0]
    goal.y=waypoints[current_goal][1]
    # Strategy is to first turn to face the target coordinates
    # and then move towards them
    while not rospy.is_shutdown() and current_goal < len(waypoints) and not stop:

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
        if dist<0.1:
            speed.angular.z=0
            speed.linear.x=0
            current_goal+=1
            if current_goal < len(waypoints):
                goal.x=waypoints[current_goal][0]
                goal.y=waypoints[current_goal][1]
	elif dist>1:
	    speed.angular.z = 0
	    speed.linear.x = 0
	    goal.x = (x + goal.x)/2
            goal.y = (x + goal.y)/2
        elif abs(angle_change) > 0.10:
            if angle_change > 0:
		speed.angular.z = 0.60
            else:
                speed.angular.z = -0.60
        else:
            speed.angular.z = 0.0
            speed.linear.x = 0.30
            rospy.loginfo("Ready to move")
               
        #print("curr: {} length_waypoint: {}".format(current_goal, len(waypoints)))
        pub.publish(speed)
        rate.sleep()
    
    speed.angular.z = 0.0
    speed.linear.x = 0.0


# Server Class
class Handler(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_POST(self):
        global last_coords
        global stop
        global turtle_thread
        global thread_arr
        
        content_length = int(self.headers['Content-Length']) # Size of Data
        data = self.rfile.read(content_length).decode('utf-8') # Data
        if data == "STOP":
            stop = True
        else:
            stop = False
            coords = json.loads(data)

            if turtle_thread != None:
                if not turtle_thread.is_alive():
                    turtle_thread = None
                else:
                    print("Bot already running")

            if not turtle_thread:
                for c in coords:
                    c[0] += last_coords[0]
                    c[1] += last_coords[1]
                
                turtle_thread = threading.Thread(target = turtle, args = (coords,)) 
                turtle_thread.start()
                last_coords = coords[-1]
        
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
    stop=True

if __name__ == '__main__':
    run()
