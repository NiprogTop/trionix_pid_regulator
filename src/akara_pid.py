#!/usr/bin/env python
from math import copysign, atan2, sin, cos, pi
import numpy as np

import rospy
from std_srvs.srv import *

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from tf.transformations import euler_from_quaternion
from pid import PID


class PidRegulator:
    def __init__(self):
        if not rospy.has_param('~heave_pid'):
            rospy.logerr("Didn't find heave pid configuration")
            raise rospy.ROSException
        self.depth_pid_ = PID(rospy.get_param('~heave_pid'))
        self.depth_setpoint_ = 0.0
        self.depth_pid_.set_setpoint(self.depth_setpoint_)
        self.depth_ = 0.0
        self.zero_pressure_ = None

        if not rospy.has_param('~roll_pid'):
            rospy.logerr("Didn't find roll pid configuration")
            raise rospy.ROSException
        self.roll_pid_ = PID(rospy.get_param('~roll_pid'))
        self.roll_pid_.set_setpoint(0.0)
        self.roll_state_ = 0.0

        if not rospy.has_param('~pitch_pid'):
            rospy.logerr("Didn't find pitch pid configuration")
            raise rospy.ROSException
        self.pitch_pid_ = PID(rospy.get_param('~pitch_pid'))
        self.pitch_pid_.set_setpoint(0.0)
        self.pitch_state_ = 0.0

        if not rospy.has_param('~yaw_pid'):
            rospy.logerr("Didn't find yaw pid configuration")
            raise rospy.ROSException
        self.yaw_pid_ = PID(rospy.get_param('~yaw_pid'))
        self.yaw_state_ = 0.0

        self.is_enabled_ = False
        self.command_ = Twist()

        self.pitch_publisher_ = rospy.Publisher('pitch', Float64, queue_size=1)
        self.wrench_publisher_ = rospy.Publisher('command', Wrench, queue_size=1)

        rospy.Subscriber('pressure', FluidPressure, self.pressure_callback)
        rospy.Subscriber('imu', Imu, self.imu_callback)
        rospy.Subscriber('teleop_command', Twist, self.vel_callback)
        rospy.Subscriber('pitch_setpoint', Float64, self.pitch_setpoint_callback)

        self.switch_service_ = rospy.Service('~switch', Trigger, self.handle_switch)
        self.update_timer_ = rospy.Timer(rospy.Duration(0.1), self.update)

    def handle_switch(self, req):
        resp = TriggerResponse()
        resp.success = True
        resp.message = 'Pid regulator ' + ('enabled', 'disabled') [self.is_enabled_]

        self.is_enabled_ = not self.is_enabled_
        self.wrench_publisher_.publish(Wrench())
        return resp

    def pressure_callback(self, msg):
        if self.zero_pressure_ is None:
            self.zero_pressure_ = msg.fluid_pressure

        self.depth_ = msg.fluid_pressure - self.zero_pressure_

    def vel_callback(self, msg):
        self.command_ = msg

    def pitch_setpoint_callback(self, msg):
        self.pitch_pid_.set_setpoint(msg.data)
        rospy.loginfo('Set pitch setpoint: %d' % msg.data)

    def imu_callback(self, msg):
        q = msg.orientation
        rpy = euler_from_quaternion([q.w, q.x, q.y, q.z])
        self.yaw_state_ = rpy[0]
        self.roll_state_ = rpy[1]
        self.pitch_state_ = rpy[2]
        self.pitch_publisher_.publish(self.pitch_state_)

    def update(self, _):
        if not self.is_enabled_:
            return

        if self.command_.linear.z:
            self.depth_setpoint_ = self.depth_setpoint_ + 0.5 * self.command_.linear.z
            if self.depth_setpoint_ < 0.0:
                self.depth_setpoint_ = 0.0
            self.depth_pid_.set_setpoint(self.depth_setpoint_)
            rospy.loginfo('depth setpoint: ' + str(self.depth_setpoint_))

        roll_effort = self.roll_pid_.update(self.roll_state_)
        heave_effort = np.clip(self.depth_pid_.update(self.depth_), -1.0, 1.0)
        pitch_effort = np.clip(self.pitch_pid_.update(self.pitch_state_), -0.5, 0.5)
        yaw_effort = self.yaw_pid_.update(self.yaw_state_)


        cmd = Wrench()
        cmd.force.x = self.command_.linear.x
        cmd.force.y = self.command_.linear.y
        cmd.force.z = heave_effort
        cmd.torque.x = self.command_.angular.x + roll_effort
        cmd.torque.y = self.command_.angular.y + pitch_effort
        cmd.torque.z = self.command_.angular.z + yaw_effort

        self.wrench_publisher_.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('pid_regulator', anonymous=True)
    p = PidRegulator()
    rospy.spin()
