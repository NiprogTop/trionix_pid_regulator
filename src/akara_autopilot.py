#!/usr/bin/env python

import rospy

from std_srvs.srv import *

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Twist, Point

from pid import PID


class Autopilot:
    def __init__(self):
        self.pids_ = {'pitch':       PID('pitch', True),
                      'surge':       PID('surge', True),
                      'sway':        PID('sway', True),
                      'heading':     PID('heading', False),
                      'depth':       PID('depth', True),
        }

        self.min_depth_ = 0.20  # meters

        self.pitch_ = 0.0
        self.heading_ = 0.0

        self.depth_ = 0.0
        self.depth_time_ = rospy.Time()

        self.basket_msg_ = Point()
        self.basket_time_ = rospy.Time()

        self.is_enabled_ = False

        self.twist_command_ = Twist()

        self.wrench_publisher_ = rospy.Publisher('command', Wrench, queue_size=1)

        rospy.Subscriber('pitch', Float64, self.pitch_callback)
        rospy.Subscriber('heading', Float64, self.heading_callback)
        rospy.Subscriber('depth', Float64, self.depth_callback)
        rospy.Subscriber('basket', Point, self.basket_callback)

        rospy.Subscriber('twist_command', Twist, self.twist_callback)

        self.switch_service_ = rospy.Service('~enable', SetBool, self.handle_switch)
        self.update_timer_ = rospy.Timer(rospy.Duration(0.1), self.update)
        self.start_time_ = rospy.Time()
        rospy.loginfo('Autopilot started')

    def handle_switch(self, req):
        self.is_enabled_ = req.data
        resp = SetBoolResponse()
        resp.success = True
        resp.message = '%s node %s' % (rospy.get_name(), ('disabled', 'enabled') [self.is_enabled_])
        self.wrench_publisher_.publish(Wrench())
        self.start_time_ = rospy.Time.now()
        return resp

    def twist_callback(self, msg):
        self.twist_command_ = msg

    def pitch_callback(self, msg):
        self.pitch_ = msg.data

    def heading_callback(self, msg):
        self.heading_ = msg.data

    def basket_callback(self, msg):
        self.basket_msg_ = msg
        self.basket_time_ = rospy.Time.now()

    def depth_callback(self, msg):
        self.depth_ = msg.data
        self.depth_time_ = rospy.Time.now()

    def get_heave_effort(self):
        if self.pids_['depth'].sp < self.min_depth_:
            return 0.0
        elif (rospy.Time.now() - self.depth_time_).secs > 0.3:
            return 0.0
        else:
            return self.pids_['depth'].update(self.depth_)

    def get_surge_and_sway_effort(self):
        if (rospy.Time.now() - self.basket_time_).secs > 0.3:
            return 0.0, 0.0
        else:
            surge = self.pids_['surge'].update(self.basket_msg_.x)
            sway = self.pids_['sway'].update(self.basket_msg_.y)
            return surge, sway

    def update(self, _):
        if not self.is_enabled_:
            return

        if (rospy.Time.now() - self.start_time_).secs >= 60 * 5:
            self.is_enabled_ = False;
            self.wrench_publisher_.publish(Wrench())
            return

        pitch_effort = self.pids_['pitch'].update(self.pitch_)
        surge_effort, sway_effort = self.get_surge_and_sway_effort()
        yaw_effort = self.pids_['heading'].update(self.heading_)
        heave_effort = self.get_heave_effort()

        # TODO translate command in world frame to body frame
        cmd_world = Wrench()
        cmd_world.force.x = self.twist_command_.linear.x + surge_effort
        cmd_world.force.y = self.twist_command_.linear.y + sway_effort
        cmd_world.force.z = self.twist_command_.linear.z + heave_effort
        cmd_world.torque.x = 0.0
        cmd_world.torque.y = pitch_effort
        cmd_world.torque.z = self.twist_command_.angular.z + yaw_effort

        self.wrench_publisher_.publish(cmd_world)


if __name__ == '__main__':
    rospy.init_node('autopilot', anonymous=True)
    a = Autopilot()
    rospy.spin()
