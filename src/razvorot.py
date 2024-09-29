#!/usr/bin/env python
from math import pi, copysign, atan2, sin, cos, radians, degrees

import rospy

from std_srvs.srv import *
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Twist, Pose
import smach
# import smach_ros

from pid import PID
from tf.transformations import euler_from_quaternion

########### CONSTS ###############

WORK_DEPTH = 0.5
SURGE_EFFORT = 0.0

EPS_DEPTH = 0.1
EPS_HEADING = radians(5)
STARTING_HEADING = radians(-10)

##################################


class Submerge(smach.State):
    def __init__(self, c, depth):
        smach.State.__init__(self, outcomes=['depth_approached'])
        self.cntr = c
        self.depth = depth

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.switch_autopilot(True)
        self.cntr.switch_depth_pid(True)
        self.cntr.switch_pitch_pid(True)
        self.cntr.switch_heading_pid(True)
        self.cntr.submerge(self.depth)
        while not rospy.is_shutdown():
            if self.cntr.depth_approached(self.depth):
                return 'depth_approached'
            else:
                self.cntr.rate.sleep()

class Emerge(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['aborted'])
        self.cntr = c

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.emerge()
        while not rospy.is_shutdown():
            if self.cntr.depth_approached(0) or ((rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(10)):
                self.cntr.switch_autopilot(False)
                self.cntr.switch_depth_pid(False)
                self.cntr.switch_pitch_pid(False)
                self.cntr.switch_heading_pid(False)
                return 'aborted'
            else:
                self.cntr.rate.sleep()

class ApproachHeadingImu(smach.State):
    def __init__(self, c, desired_heading):
        smach.State.__init__(self, outcomes=['heading_approached'])
        self.cntr = c
        self.desired_heading = desired_heading

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.set_heading(self.desired_heading)
        while not rospy.is_shutdown():
            if self.cntr.heading_approached(self.desired_heading):
                return 'heading_approached'
            else:
                self.cntr.rate.sleep()


class MoveStraight(smach.State):
    def __init__(self, c, time):
        smach.State.__init__(self, outcomes=['timeout'])
        self.cntr = c
        self.time = time

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.move(SURGE_EFFORT, 0.0)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(self.time):
                self.cntr.move(0.0, 0.0)
                return 'timeout'
            else:
                self.cntr.rate.sleep()


class Controller():
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.heading_eps_ = EPS_HEADING
        self.depth_ = 0.0
        self.depth_eps_ = EPS_DEPTH
        self.heading_ = 0.0
        self.working_depth_ = WORK_DEPTH

        rospy.Subscriber('depth', Float64, self.depth_callback)
        rospy.Subscriber('heading', Float64, self.heading_callback)

        rospy.loginfo('waiting sevices')
        rospy.wait_for_service('/autopilot/enable')
        rospy.wait_for_service('/autopilot/depth_pid/enable')
        rospy.wait_for_service('/autopilot/pitch_pid/enable')
        rospy.wait_for_service('/autopilot/heading_pid/enable')
        self.autopilot_enable_service = rospy.ServiceProxy('/autopilot/enable', SetBool)
        self.depth_pid_enable_service = rospy.ServiceProxy('/autopilot/depth_pid/enable', SetBool)
        self.pitch_pid_enable_service = rospy.ServiceProxy('/autopilot/pitch_pid/enable', SetBool)
        self.heading_pid_enable_service = rospy.ServiceProxy('/autopilot/heading_pid/enable', SetBool)

        self.command_publisher_ = rospy.Publisher('twist_command', Twist, queue_size=1)
        self.pose_publisher_ = rospy.Publisher('pose', Pose, queue_size=1)
        self.depth_sp_publisher = rospy.Publisher('depth_pid/setpoint', Float64, queue_size=1)
        self.heading_sp_publisher = rospy.Publisher('heading_pid/setpoint', Float64, queue_size=1)

        self.state_change_time = rospy.Time.now()

    def set_heading(self, heading):
        self.heading_sp_publisher.publish(heading)

    def submerge(self, depth):
        self.depth_sp_publisher.publish(self.working_depth_)

    def switch_autopilot(self, en):
        resp = self.autopilot_enable_service(en)

    def switch_depth_pid(self, en):
        resp = self.depth_pid_enable_service(en)

    def switch_pitch_pid(self, en):
        resp = self.pitch_pid_enable_service(en)

    def switch_heading_pid(self, en):
        resp = self.heading_pid_enable_service(en)

    def emerge(self):
        self.depth_sp_publisher.publish(0)

    def heading_approached(self, d):
        # e = radians((degrees(self.heading_ - d) + 180) % 360 - 180)
        if abs(self.heading_ - d) < self.heading_eps_:
            return True
        else:
            return False

    def depth_approached(self, d):
        if abs(d - self.depth_) < self.depth_eps_:
            return True
        else:
            return False

    def heading_callback(self, msg):
        self.heading_ = msg.data

    def depth_callback(self, msg):
        self.depth_ = msg.data

    def move(self, x, y):
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        self.command_publisher_.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('akara_state_maschine')

    c = Controller()
    sm = smach.StateMachine(outcomes=['aborted'])
    with sm:
        smach.StateMachine.add('SUBMERGE', Submerge(c, WORK_DEPTH),
                               transitions={'depth_approached': 'HEADING1'})

        smach.StateMachine.add('HEADING1', ApproachHeadingImu(c, 0.0),
                               transitions={'heading_approached': 'HEADING2'})

        smach.StateMachine.add('HEADING2', ApproachHeadingImu(c, 1.6),
                               transitions={'heading_approached': 'HEADING3'})

        smach.StateMachine.add('HEADING3', ApproachHeadingImu(c, 3.1),
                               transitions={'heading_approached': 'HEADING4'})

        smach.StateMachine.add('HEADING4', ApproachHeadingImu(c, -1.6),
                               transitions={'heading_approached': 'HEADING5'})

        smach.StateMachine.add('HEADING5', ApproachHeadingImu(c, 0.0),
                               transitions={'heading_approached': 'EMERGE'})
        # smach.StateMachine.add('WAIT', MoveStraight(c, 10),
        #                        transitions={'timeout': 'EMERGE'})

        smach.StateMachine.add('EMERGE', Emerge(c),
                               transitions={'aborted': 'aborted'})


    rospy.sleep(rospy.Duration(1))
    outcome = sm.execute()
