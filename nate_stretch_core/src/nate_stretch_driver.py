#! /usr/bin/env python
from __future__ import print_function

import yaml
import numpy as np
import threading
from rwlock import RWLock
import stretch_body.robot as rb
from stretch_body.hello_utils import ThreadServiceExit

import tf2_ros
import tf_conversions

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryResult

from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, MagneticField
from std_msgs.msg import Header

from joint_trajectory_server import JointTrajectoryAction


class StretchBodyNode:

    def __init__(self):
        self.use_robotis_head = True
        self.use_robotis_end_of_arm = True

        self.default_goal_timeout_s = 10.0
        self.default_goal_timeout_duration = rospy.Duration(self.default_goal_timeout_s)

        self.mobile_base_manipulation_origin = None #{'x':None, 'y':None, 'theta':None}

        self.robot_stop_lock = threading.Lock()
        self.stop_the_robot = False

        self.robot_mode_rwlock = RWLock()
        self.robot_mode = None

    ###### MOBILE BASE VELOCITY METHODS #######

    def set_mobile_base_velocity_callback(self, twist):
        self.robot_mode_rwlock.acquire_read()
        if self.robot_mode != 'navigation':
            error_string = '{0} action server must be in navigation mode to receive a twist on cmd_vel. Current mode = {1}.'.format(self.node_name, self.robot_mode)
            rospy.logerr(error_string)
            return
        self.linear_velocity_mps = twist.linear.x
        self.angular_velocity_radps = twist.angular.z
        self.last_twist_time = rospy.get_time()
        self.robot_mode_rwlock.release_read()

    def command_mobile_base_velocity_and_publish_state(self):
        self.robot_mode_rwlock.acquire_read()

        # set new mobile base velocities if available
        if self.robot_mode == 'navigation': 
            time_since_last_twist = rospy.get_time() - self.last_twist_time
            if time_since_last_twist < self.timeout:
                self.robot.base.set_velocity(self.linear_velocity_mps, self.angular_velocity_radps)
                self.robot.push_command()
            else:
                # Watchdog timer stops motion if no communication within timeout
                self.robot.base.set_velocity(0.0, 0.0)
                self.robot.push_command()

        # TODO: In the future, consider using time stamps from the robot's
        # motor control boards and other boards. These would need to
        # be synchronized with the rospy clock.
        #robot_time = robot_status['timestamp_pc']
        #current_time = rospy.Time.from_sec(robot_time)
        current_time = rospy.Time.now()
        robot_status = self.robot.get_status()

        # obtain odometry
        base_status = robot_status['base']
        x = base_status['x']
        y = base_status['y']
        theta = base_status['theta']
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        x_vel = base_status['x_vel']
        x_effort = base_status['effort'][0]
        theta_vel = base_status['theta_vel']
        pose_time_s = base_status['pose_time_s']

        if self.robot_mode == 'manipulation':
            x = self.mobile_base_manipulation_origin['x']
            x_vel = 0.0
            x_effort = 0.0

        if self.broadcast_odom_tf:
            # publish odometry via TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.angular.z = theta_vel
        self.odom_pub.publish(odom)

        # publish joint state
        joint_state = JointState()
        joint_state.header.stamp = current_time
        for cg in self.joint_trajectory_action.command_groups:
            pos, vel, effort = cg.joint_state(robot_status, robot_mode=self.robot_mode,
                               manipulation_origin=self.mobile_base_manipulation_origin)
            joint_state.name.append(cg.name)
            joint_state.position.append(pos)
            joint_state.velocity.append(vel)
            joint_state.effort.append(effort)

        # add telescoping joints to joint state
        arm_cg = self.joint_trajectory_action.arm_cg
        joint_state.name.extend(arm_cg.telescoping_joints)
        pos, vel, effort = arm_cg.joint_state(robot_status)
        for _ in range(len(arm_cg.telescoping_joints)):
            joint_state.position.append(pos / len(arm_cg.telescoping_joints))
            joint_state.velocity.append(vel / len(arm_cg.telescoping_joints))
            joint_state.effort.append(effort)

        # add gripper joints to joint state
        gripper_cg = self.joint_trajectory_action.gripper_cg
        if gripper_cg is not None:
            missing_gripper_joint_names = list(set(gripper_cg.gripper_joint_names) - set(joint_state.name))
            for j in missing_gripper_joint_names:
                pos, vel, effort = gripper_cg.joint_state(robot_status, joint_name=j)
                joint_state.name.append(j)
                joint_state.position.append(pos)
                joint_state.velocity.append(vel)
                joint_state.effort.append(effort)

        self.joint_state_pub.publish(joint_state)

        ##################################################
        # publish IMU sensor data
        imu_status = robot_status['pimu']['imu']
        ax = imu_status['ax']
        ay = imu_status['ay']
        az = imu_status['az']
        gx = imu_status['gx']
        gy = imu_status['gy']
        gz = imu_status['gz']
        mx = imu_status['mx']
        my = imu_status['my']
        mz = imu_status['mz']

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = 'imu_mobile_base'
        i.angular_velocity.x = gx
        i.angular_velocity.y = gy
        i.angular_velocity.z = gz
        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_mobile_base_pub.publish(i)

        m = MagneticField()
        m.header.stamp = current_time
        m.header.frame_id = 'imu_mobile_base'
        self.magnetometer_mobile_base_pub.publish(m)

        accel_status = robot_status['wacc']
        ax = accel_status['ax']
        ay = accel_status['ay']
        az = accel_status['az']

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = 'accel_wrist'
        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_wrist_pub.publish(i)
        ##################################################

        self.robot_mode_rwlock.release_read()

    ######## CHANGE MODES #########

    def change_mode(self, new_mode, code_to_run):
        self.robot_mode_rwlock.acquire_write()
        self.robot_mode = new_mode
        code_to_run()
        rospy.loginfo('{0}: Changed to mode = {1}'.format(self.node_name, self.robot_mode))
        self.robot_mode_rwlock.release_write()

    # TODO : add a freewheel mode or something comparable for the mobile base?

    def turn_on_navigation_mode(self):
        # Navigation mode enables mobile base velocity control via
        # cmd_vel, and disables position-based control of the mobile
        # base.
        def code_to_run():
            self.linear_velocity_mps = 0.0
            self.angular_velocity_radps = 0.0
        self.change_mode('navigation', code_to_run)

    def turn_on_manipulation_mode(self):
        # Manipulation mode enables mobile base translation using
        # position control via joint_mobile_base_translation, and
        # disables velocity control of the mobile base. It also
        # updates the virtual prismatic joint named
        # joint_mobile_base_translation that relates 'floor_link' and
        # 'base_link'. This mode was originally created so that
        # MoveIt! could treat the robot like an arm. This mode does
        # not allow base rotation.
        def code_to_run():
            self.robot.base.enable_pos_incr_mode()
            # get copy of the current robot status (uses lock held by the robot)
            robot_status = self.robot.get_status()
            # obtain odometry
            # assign relevant base status to variables
            base_status = robot_status['base']
            x = base_status['x']
            y = base_status['y']
            theta = base_status['theta']
            self.mobile_base_manipulation_origin = {'x':x, 'y':y, 'theta':theta} 
        self.change_mode('manipulation', code_to_run)

    def turn_on_position_mode(self):
        # Position mode enables mobile base translation and rotation
        # using position control with sequential incremental rotations
        # and translations. It also disables velocity control of the
        # mobile base. It does not update the virtual prismatic
        # joint. The frames associated with 'floor_link' and
        # 'base_link' become identical in this mode.
        def code_to_run():
            self.robot.base.enable_pos_incr_mode()
        self.change_mode('position', code_to_run)

    def calibrate(self):
        def code_to_run():
            self.robot.lift.home()
            self.robot.arm.home()
        self.change_mode('calibration', code_to_run)

    ######## SERVICE CALLBACKS #######

    def stop_the_robot_callback(self, request):
        with self.robot_stop_lock:
            self.stop_the_robot = True

            self.robot.base.translate_by(0.0)
            self.robot.base.rotate_by(0.0)
            self.robot.arm.move_by(0.0)
            self.robot.lift.move_by(0.0)
            self.robot.push_command()

            for joint in self.robot.head.joints:
                self.robot.head.move_by(joint, 0.0)
            for joint in self.robot.end_of_arm.joints:
                self.robot.end_of_arm.move_by(joint, 0.0)

        rospy.loginfo('Received stop_the_robot service call, so commanded all actuators to stop.')
        return TriggerResponse(
            success=True,
            message='Stopped the robot.'
            )

    def navigation_mode_service_callback(self, request):
        self.turn_on_navigation_mode()
        return TriggerResponse(
            success=True,
            message='Now in navigation mode.'
            )

    def manipulation_mode_service_callback(self, request):
        self.turn_on_manipulation_mode()
        return TriggerResponse(
            success=True,
            message='Now in manipulation mode.'
            )

    def position_mode_service_callback(self, request):
        self.turn_on_position_mode()
        return TriggerResponse(
            success=True,
            message='Now in position mode.'
            )

    def runstop_service_callback(self, request):
        if request.data:
            with self.robot_stop_lock:
                self.stop_the_robot = True

                self.robot.base.translate_by(0.0)
                self.robot.base.rotate_by(0.0)
                self.robot.arm.move_by(0.0)
                self.robot.lift.move_by(0.0)
                self.robot.push_command()

                for joint in self.robot.head.joints:
                    self.robot.head.move_by(joint, 0.0)
                for joint in self.robot.end_of_arm.joints:
                    self.robot.end_of_arm.move_by(joint, 0.0)

            self.robot.pimu.runstop_event_trigger()
        else:
            self.robot.pimu.runstop_event_reset()

        return SetBoolResponse(
            success=True,
            message='is_runstopped: {0}'.format(request.data)
            )

    ########### MAIN ############

    def main(self):

        rospy.init_node('stretch_driver')
        self.node_name = rospy.get_name()

        rospy.loginfo("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")

        rospy.loginfo("{0} started".format(self.node_name))

        self.robot = rb.Robot()
        self.robot.startup()

        mode = rospy.get_param('~mode', "position")
        rospy.loginfo('mode = ' + str(mode))
        if mode == "position":
            self.turn_on_position_mode()
        elif mode == "navigation":
            self.turn_on_navigation_mode()
        elif mode == "manipulation":
            self.turn_on_manipulation_mode()

        self.broadcast_odom_tf = rospy.get_param('~broadcast_odom_tf', False)
        rospy.loginfo('broadcast_odom_tf = ' + str(self.broadcast_odom_tf))
        if self.broadcast_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        large_ang = np.radians(45.0)
        filename = rospy.get_param('~controller_calibration_file')
        rospy.loginfo('Loading controller calibration parameters for the head from YAML file named {0}'.format(filename))
        with open(filename, 'r') as fid:
            self.controller_parameters = yaml.safe_load(fid)
            rospy.loginfo('controller parameters loaded = {0}'.format(self.controller_parameters))

            head_tilt_calibrated_offset_rad = self.controller_parameters['tilt_angle_offset']
            if (abs(head_tilt_calibrated_offset_rad) > large_ang):
                rospy.logwarn('WARNING: head_tilt_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            rospy.loginfo('head_tilt_calibrated_offset_rad in degrees = {0}'.format(np.degrees(head_tilt_calibrated_offset_rad)))

            head_pan_calibrated_offset_rad = self.controller_parameters['pan_angle_offset']
            if (abs(head_pan_calibrated_offset_rad) > large_ang):
                rospy.logwarn('WARNING: head_pan_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            rospy.loginfo('head_pan_calibrated_offset_rad in degrees = {0}'.format(np.degrees(head_pan_calibrated_offset_rad)))

            head_pan_calibrated_looked_left_offset_rad = self.controller_parameters['pan_looked_left_offset']
            if (abs(head_pan_calibrated_looked_left_offset_rad) > large_ang):
                rospy.logwarn('WARNING: head_pan_calibrated_looked_left_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            rospy.loginfo('head_pan_calibrated_looked_left_offset_rad in degrees = {0}'.format(np.degrees(head_pan_calibrated_looked_left_offset_rad)))

            head_tilt_backlash_transition_angle_rad = self.controller_parameters['tilt_angle_backlash_transition']
            rospy.loginfo('head_tilt_backlash_transition_angle_rad in degrees = {0}'.format(np.degrees(head_tilt_backlash_transition_angle_rad)))

            head_tilt_calibrated_looking_up_offset_rad = self.controller_parameters['tilt_looking_up_offset']
            if (abs(head_tilt_calibrated_looking_up_offset_rad) > large_ang):
                rospy.logwarn('WARNING: head_tilt_calibrated_looking_up_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            rospy.loginfo('head_tilt_calibrated_looking_up_offset_rad in degrees = {0}'.format(np.degrees(head_tilt_calibrated_looking_up_offset_rad)))

            arm_calibrated_retracted_offset_m = self.controller_parameters['arm_retracted_offset']
            if (abs(arm_calibrated_retracted_offset_m) > 0.05):
                rospy.logwarn('WARNING: arm_calibrated_retracted_offset_m HAS AN UNUSUALLY LARGE MAGNITUDE')
            rospy.loginfo('arm_calibrated_retracted_offset_m in meters = {0}'.format(arm_calibrated_retracted_offset_m))

        self.linear_velocity_mps = 0.0 # m/s ROS SI standard for cmd_vel (REP 103)
        self.angular_velocity_radps = 0.0 # rad/s ROS SI standard for cmd_vel (REP 103)

        self.max_arm_height = 1.1

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

        self.imu_mobile_base_pub = rospy.Publisher('imu_mobile_base', Imu, queue_size=1)
        self.magnetometer_mobile_base_pub = rospy.Publisher('magnetometer_mobile_base', MagneticField, queue_size=1)
        self.imu_wrist_pub = rospy.Publisher('imu_wrist', Imu, queue_size=1)

        rospy.Subscriber("cmd_vel", Twist, self.set_mobile_base_velocity_callback)

        # ~ symbol gets parameter from private namespace
        self.joint_state_rate = rospy.get_param('~rate', 15.0)
        self.timeout = rospy.get_param('~timeout', 1.0)
        rospy.loginfo("{0} rate = {1} Hz".format(self.node_name, self.joint_state_rate))
        rospy.loginfo("{0} timeout = {1} s".format(self.node_name, self.timeout))

        self.use_fake_mechaduinos = rospy.get_param('~use_fake_mechaduinos', False)
        rospy.loginfo("{0} use_fake_mechaduinos = {1}".format(rospy.get_name(), self.use_fake_mechaduinos))

        self.base_frame_id = 'base_link'
        rospy.loginfo("{0} base_frame_id = {1}".format(self.node_name, self.base_frame_id))
        self.odom_frame_id = 'odom'
        rospy.loginfo("{0} odom_frame_id = {1}".format(self.node_name, self.odom_frame_id))

        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

        command_base_velocity_and_publish_joint_state_rate = rospy.Rate(self.joint_state_rate)
        self.last_twist_time = rospy.get_time()

        # start action server for joint trajectories
        self.fail_out_of_range_goal = rospy.get_param('~fail_out_of_range_goal', True)
        self.joint_trajectory_action = JointTrajectoryAction(self)
        self.joint_trajectory_action.server.start()

        self.switch_to_manipulation_mode_service = rospy.Service('/switch_to_manipulation_mode',
                                                                 Trigger,
                                                                 self.manipulation_mode_service_callback)

        self.switch_to_navigation_mode_service = rospy.Service('/switch_to_navigation_mode',
                                                               Trigger,
                                                               self.navigation_mode_service_callback)

        self.switch_to_position_mode_service = rospy.Service('/switch_to_position_mode',
                                                             Trigger,
                                                             self.position_mode_service_callback)
        
        self.stop_the_robot_service = rospy.Service('/stop_the_robot',
                                                    Trigger,
                                                    self.stop_the_robot_callback)

        self.runstop_service = rospy.Service('/runstop',
                                              SetBool,
                                              self.runstop_service_callback)

        try:
            # start loop to command the mobile base velocity, publish
            # odometry, and publish joint states
            while not rospy.is_shutdown():
                self.command_mobile_base_velocity_and_publish_state()
                command_base_velocity_and_publish_joint_state_rate.sleep()
        except (rospy.ROSInterruptException, ThreadServiceExit):
            self.robot.stop()
            rospy.signal_shutdown("stretch_driver shutdown")


if __name__ == '__main__':
    node = StretchBodyNode()
    node.main()
