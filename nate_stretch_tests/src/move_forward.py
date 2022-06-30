#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class Move:
	"""
	A class that sends Twist messages to move the Stretch robot foward.
	"""
	def __init__(self):
		"""
		Function that initializes the subscriber.
		:param self: The self reference
		"""
		self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo

	def move_forward(self):
		"""
		Function that publishes Twist messages
		:param self: The self reference.

		:publishes command: Twist message
		"""
		command = Twist()
		command.linear.x = 0.1
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0
		self.pub.publish(command)

if __name__ == '__main__':
	rospy.init_node('move')
	base_motion = Move()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		base_motion.move_forward()
		rate.sleep()
