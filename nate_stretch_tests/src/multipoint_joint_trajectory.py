#!/usr/bin/env python

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm

class MultiPointCommand(hm.HelloNode):
  """
  A class that sends multiple joint trajectory goals to the stretch robot.
  """
  def __init__(self):
    hm.HelloNode.__init__(self)

  def issue_multipoint_command(self):
    """
    Function that makes an action call and sends multiple joint trajectory goals
    to the joint_lift, wrist_extension, and joint_wrist_yaw.
    :param self: The self reference.
    """
    point0 = JointTrajectoryPoint()
    point0.positions = [0.2, 0.0, 3.4]
    point0.velocities = [0.2, 0.2, 2.5]
    point0.accelerations = [1.0, 1.0, 3.5]

    point1 = JointTrajectoryPoint()
    point1.positions = [0.3, 0.1, 2.0]

    point2 = JointTrajectoryPoint()
    point2.positions = [0.5, 0.2, -1.0]

    point3 = JointTrajectoryPoint()
    point3.positions = [0.6, 0.3, 0.0]

    point4 = JointTrajectoryPoint()
    point4.positions = [0.8, 0.2, 1.0]

    point5 = JointTrajectoryPoint()
    point5.positions = [0.5, 0.1, 0.0]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
    trajectory_goal.trajectory.points = [point0, point1, point2, point3, point4, point5]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def main(self):
    """
    Function that initiates the multipoint_command function.
    :param self: The self reference.
    """
    hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
    rospy.loginfo('issuing multipoint command...')
    self.issue_multipoint_command()
    time.sleep(2)


if __name__ == '__main__':
  try:
    node = MultiPointCommand()
    node.main()
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')
