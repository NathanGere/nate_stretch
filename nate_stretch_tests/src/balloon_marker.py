#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class Balloon():
	"""
	A class that attaches a Sphere marker directly above the Stretch robot.
	"""
	def __init__(self):
		"""
		Function that initializes the markers features.
		:param self: The self reference
		"""
		self.publisher = rospy.Publisher('balloon', Marker, queue_size=10) #defines talker's interface. ROS. pub = rospy.Publisher("balloon", Twist, queue_size=1) declares that your node is publishing to the /ballon topic using the message type Twist.
		
		#creating a marker with position base link and a sphere shape
		self.marker = Marker()
		self.marker.header.frame_id = '/base_link'
		self.marker.header.stamp = rospy.Time()
		self.marker.type = self.marker.SPHERE
	
		#giving the marker a unique ID number
		self.marker.id = 0

		#sets the action
		self.marker.action = self.marker.ADD

		#size of the marker
		self.marker.scale.x = 0.5
		self.marker.scale.y = 0.5
		self.marker.scale.z = 0.5

		#color of the marker
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0

		#transparency of the marker
		self.marker.color.a = 1.0

		#setting marker pose in reference to base link. since its a sphere, rotation doesn't matter. this marker will be 2 meters above base link at all times
		self.marker.pose.position.x = 0.0
		self.marker.pose.position.y = 0.0
		self.marker.pose.position.z = 2.0

	def publish_marker(self):
		"""
		Function that publishes the sphere marker
		:param self: The self reference

		:publishes self.marker: Marker message
		"""
		self.publisher.publish(self.marker) #publishing marker to be visable in rviz


if __name__ == '__main__':
	rospy.init_node('marker', argv=sys.argv)
	ballon = Balloon()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		ballon.publish_marker()
		rate.sleep()		
