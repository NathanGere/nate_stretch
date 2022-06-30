#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState

class JointStatePublisher():
	"""
	A class that prints the positions of desired joints in Stretch.
	"""
	def __init__(self):
		"""
		Function that initializes the subsriber.
		:param self: The self reference
		"""
		self.sub = rospy.Subscriber('joint_states', JointState, self.callback)

	def callback(self, msg):
		"""
		Callback function to deal with the incoming JointState messages.
		:param self: The self reference
		:param msg: The JointState message.
		"""
		self.joint_states = msg

	def print_states(self, joints):
		"""
		print_states function to deal with the incoming JointState messages.
		:param self: The self reference.
		:param joints: A list of joint names
		"""
		joint_positions = [] #empty list where joint positions will be appended

		#iterates through to find joint positions
		for i in range(len(joints)):
			index = self.joint_states.name.index(joints[i])
			joint_positions.append(self.joint_states.position[index])
		print("name: " + str(joints))
		print("position: " + str(joint_positions))
		rospy.signal_shutdown("done") #shutsdown ros
		sys.exit(0) #exits python interpretor

if __name__ == '__main__':
	rospy.init_node('joint_state_printer', anonymous=True)
	JSP = JointStatePublisher()
	rospy.sleep(.1)
	joints = ["joint_lift", "joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_13", "joint_wrist_yaw"]
	#joints = ["joint_head_pan","joint_head_tilt", joint_gripper_finger_left", "joint_gripper_finger_right"]
	JSP.print_states(joints)
	rospy.spin()
