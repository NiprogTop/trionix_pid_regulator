#!/usr/bin/env python

import rospy

from std_srvs.srv import SetBool, SetBoolResponse

from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Twist

from pid import PID


class Pid:
    min_depth_ = 0.20  # meters
    pitch_ = 0.0
    depth_ = 0.0
    running_ = False

    def __init__(self):
        self.pids_ = {'pitch': PID('pitch', enabled=True),
                      'depth': PID('depth', enabled=True)}

        self.depth_time_ = rospy.Time()
        self.twist_command_ = Twist()
        self.wrench_publisher_ = rospy.Publisher('command', Wrench, queue_size=1)

        rospy.Subscriber('pitch', Float64, self.pitch_callback)
        rospy.Subscriber('depth', Float64, self.depth_callback)
        rospy.Subscriber('teleop_command', Twist, self.twist_callback)

        self.switch_service_ = rospy.Service('~enable', SetBool, self.handle_switch)
        self.update_timer_ = rospy.Timer(rospy.Duration(0.1), self.update)
        self.start_time_ = rospy.Time()

        rospy.loginfo('Pid started')

    def handle_switch(self, req):
        self.running_ = req.data
        resp = SetBoolResponse()
        resp.success = True
        resp.message = '%s node %s' % (rospy.get_name(), ('disabled', 'enabled')[self.running_])
        self.wrench_publisher_.publish(Wrench())
        self.start_time_ = rospy.Time.now()
        return resp

    def twist_callback(self, msg):
        self.twist_command_ = msg

    def pitch_callback(self, msg):
        self.pitch_ = msg.data

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

    def update(self, _):
        # if not self.running_:
        #     return

        # if (rospy.Time.now() - self.start_time_).secs >= 60 * 5:
        #     self.running_ = False
        #     self.wrench_publisher_.publish(Wrench())
        #     return

        # TODO translate command in world frame to body frame
        cmd_world = Wrench()
        cmd_world.force.x = self.twist_command_.linear.x
        cmd_world.force.y = self.twist_command_.linear.y
        cmd_world.force.z = self.twist_command_.linear.z + self.get_heave_effort()
        cmd_world.torque.y = self.pids_['pitch'].update(self.pitch_)
        cmd_world.torque.z = self.twist_command_.angular.z

        self.wrench_publisher_.publish(cmd_world)


if __name__ == '__main__':
    rospy.init_node('Pid', anonymous=True)
    p = Pid()
    rospy.spin()
