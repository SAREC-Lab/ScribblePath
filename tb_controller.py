#!/usr/bin/env python
# Software License Agreement (BSD License)
# Modified from https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
# to work with TurtleBot3-Burger

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, pi

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
    # waypoints = [[5,5],[-6,1],[3,8],[10,0]]
    goal.x=waypoints[current_goal][0]
    goal.y=waypoints[current_goal][1]
    # Strategy is to first turn to face the target coordinates
    # and then move towards them
    while not rospy.is_shutdown():

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
        if dist<.1:
            speed.angular.z=0
            speed.linear.x=0
            if current_goal +1 < len(waypoints):
                current_goal+=1
                goal.x=waypoints[current_goal][0]
                goal.y=waypoints[current_goal][1]
        elif abs(angle_change) > 0.25:
            #speed.linear.x = 0.0
            if angle_change > 0:
                speed.angular.z = 0.50

                if angle_change > pi/8:
                    speed.angular.z = 0.75

                if angle_change > pi/4:
                    speed.angular.z = 1.00

                if angle_change > pi/2:
                    speed.angular.z = 1.25

                if angle_change > 3*pi/4:
                    speed.angular.z = 1.50

                if angle_change > 3*pi/4 + ((3*pi/4) + pi)/2:
                    speed.angular.z = 1.75
            else:
                speed.angular.z = -0.50

                if angle_change < -pi/8:
                    speed.angular.z = -0.75

                if angle_change < -pi/4:
                    speed.angular.z = -1.00

                if angle_change < -pi/2:
                    speed.angular.z = -1.25

                if angle_change < -3*pi/4:
                    speed.angular.z = -1.50

                if angle_change < -(3*pi/4 + ((3*pi/4) + pi)/2):
                    speed.angular.z = -1.70
        else:
            speed.angular.z = 0.0

            rospy.loginfo("Ready to move")
        
            speed.linear.x = 0.300
    
            if dist > 0.25:
                speed.linear.x = 0.350

            if dist > 0.50:
                speed.linear.x = 0.400

            if dist > 0.75:
                speed.linear.x = 0.450
            
            if dist > 1:
                speed.linear.x = 0.500
            
            if dist > 1.25:
                speed.linear.x = 0.550
            
            if dist > 1.5:
                speed.linear.x = 0.600

            # speed.linear.x = dist*0.05+.025
            # speed.angular.z = 0.0
        
        pub.publish(speed)
        r.sleep()

def main():
    waypoints = [[1,1], [1,1.1], [1,1.2], [2,0], [3,1], [4,0]]
    turtle(waypoints)

if __name__ == "__main__":
    main()
