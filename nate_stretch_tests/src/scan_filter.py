#!/usr/bin/env python

import rospy
from numpy import linspace, inf
from math import sin
from sensor_msgs.msg import LaserScan

class Scanfilter:
  """
  A class that implements a LaserScan filter that removes all of the points
  that are not infront of the robot
  """
	def __init__(self):
		self.width = 1.0
		self.extent = self.width / 2.0
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		self.pub = rospy.Publisher('filtered_scan', LaserScan, queue_size=10)

	def callback(self,msg):
    """
		Callback function to deal with incoming laserscan messages.
		:param self: The self reference.
		:param msg: The subscribed laserscan message.

		:publishes msg: updated laserscan message.
		"""
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
		points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]
		new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]
		msg.ranges = new_ranges
		self.pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('scan_filter')
	Scanfilter()
	rospy.spin()
