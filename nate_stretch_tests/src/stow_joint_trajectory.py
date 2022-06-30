#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
import time

class StowCommand(hm.HelloNode):
  '''
  A class that sends a joint trajectory goal to stow the Stretch's arm.
  '''
  def __init__(self):
    hm.HelloNode.__init__(self)

  def issue_stow_command(self):
    '''
    Function that makes an action call and sends stow postion goal.
    :param self: The self reference.
    '''
    stow_point = JointTrajectoryPoint() 
    stow_point.time_from_start = rospy.Duration(0.000)
    stow_point.positions = [0.2, 0.0, 3.4]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
    trajectory_goal.trajectory.points = [stow_point]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def main(self):
    '''
    Function that initiates stow_command function.
    :param self: The self reference.
    '''
    hm.HelloNode.main(self, 'stow_command', 'stow_command', wait_for_first_pointcloud=False)
    rospy.loginfo('stowing...')
    self.issue_stow_command()
    time.sleep(2)


if __name__ == '__main__':
  try:
    node = StowCommand()
    node.main()
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')
