#!/usr/bin/env python

import rospy
from numpy import linspace, inf, tanh
from math import sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Avoider:
    """
    A class that implements both a LaserScan filter and base velocity control
    for collision avoidance.
    """
    def __init__(self):
        """
        Function that initializes the subscriber, publisher, and marker features.
        :param self: The self reference
        """
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo
        self.sub = rospy.Subscriber('/scan', LaserScan, self.set_speed) #to retrieve scan values
	
        self.width = 1 #setting width of laser scan values
        self.extent = self.width / 2.0	#points this set extent from the x-axis will not be evaluated
        self.distance = 0.5 #setting stopping distance from an object
	
	#allocating twist with values set to zero for initialization
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def set_speed(self,msg):
        """
        Callback function to deal with incoming laserscan messages.
        :param self: The self reference.
        :param msg: The subscribed laserscan message.

        :publishes self.twist: Twist message.
        """
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)] 
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

        error = min(new_ranges) - self.distance #calc distance between closest measured scan and stopping point

        self.twist.linear.x = tanh(error) if (error > 0.05 or error < -0.05) else 0 #set speed according to tan fxn
        self.pub.publish(self.twist)		

if __name__ == '__main__':
    rospy.init_node('avoider')
    Avoider()
    rospy.spin()
