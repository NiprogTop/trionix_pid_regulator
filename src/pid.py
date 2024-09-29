#!/usr/bin/env python

from math import pi

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64


class PID:
    kp = 0.0
    ki = 0.0
    kd = 0.0
    clamp = 0.0
    angular = False
    inverted = False

    sp = 0.0
    e_sum = 0.0
    e_old = 0.0

    def __init__(self, name, enabled=False):
        self.name_ = name

        if rospy.has_param('~%s_pid' % self.name_):
            s = rospy.get_param('~%s_pid' % self.name_)
            self.set_params(s)
        else:
            rospy.logwarn("Didn't find %s pid configuration. Assign zeros." % self.name_)

        self.enabled = enabled
        rospy.Subscriber('%s_pid/setpoint' % self.name_, Float64, self.sp_callback)
        self.switch_service = rospy.Service('~%s_pid/enable' % self.name_, SetBool,
                                            self.handle_switch_service)

    def set_params(self, settings):
        self.kp = settings['kp']
        self.ki = settings['ki']
        self.kd = settings['kd']
        self.angular = settings['angular']
        self.clamp = settings['clamp']
        self.inverted = settings['inverted']

    def sp_callback(self, msg):
        self.sp = msg.data
        self.e_sum = 0.0
        self.e_old = 0.0

    def handle_switch_service(self, req):
        self.enable(req.data)
        resp = SetBoolResponse()
        resp.success = True
        resp.message = '%s pid %s' % (self.name_, ('disabled', 'enabled')[self.enabled])
        return resp

    def enable(self, state):
        self.enabled = state
        if self.enabled:
            self.e_sum = self.e_old = 0.0

    def angular_constraint(self, e):
        return (e + pi) % (2 * pi) - pi

    def update(self, state):
        if not self.enabled:
            return 0.0

        error = self.sp - state if not self.inverted else state - self.sp
        if self.angular:
            error = self.angular_constraint(error)
        p = self.kp * error
        self.e_sum += self.ki * error
        self.e_sum = min(self.clamp, max(self.e_sum, -self.clamp))
        d = self.kd * (error - self.e_old)
        self.e_old = error
        PID = p + self.e_sum + d
        return min(self.clamp, max(PID, -self.clamp))
