#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Wrench

from pid import PID


def clip_abs(v, v_max):
    return max(-v_max, min(v, v_max))

class PidRegulator:
    def __init__(self):
        self.is_enabled_ = False
        self.twist_ = Twist()

        self.roll_pid_ = PID('roll', True)
        self.roll_state_ = 0.0

        self.pitch_pid_ = PID('pitch', True)
        self.pitch_state_ = 0.0

        self.heading_pid_ = PID('heading', True)
        self.heading_state_ = 0.0

        self.depth_pid_ = PID('depth', False)
        self.depth_state_ = 0.0

        self.max_effort_ = abs(rospy.get_param("~max_effort", 1.0))

        rospy.Subscriber('roll', Float64, self.roll_callback)
        rospy.Subscriber('pitch', Float64, self.pitch_callback)
        rospy.Subscriber('heading', Float64, self.heading_callback)
        rospy.Subscriber('depth', Float64, self.depth_callback)

        rospy.Subscriber('teleop_command', Twist, self.vel_callback)

        self.wrench_publisher_ = rospy.Publisher('command', Wrench, queue_size=1)
        self.switch_service_ = rospy.Service('~switch', Trigger, self.handle_switch)
        self.update_timer_ = rospy.Timer(rospy.Duration(0.1), self.update)
        self.command_ = Twist()

    def handle_switch(self, req):
        self.is_enabled_ = not self.is_enabled_
        resp = TriggerResponse()
        resp.success = True
        resp.message = '%s node %s' % (rospy.get_name(), ('disabled', 'enabled')[self.is_enabled_])
        self.wrench_publisher_.publish(Wrench())
        return resp

    def update(self, _):
        if not self.is_enabled_:
            return

        roll_effort = self.roll_pid_.update(self.roll_state_)
        pitch_effort = self.pitch_pid_.update(self.pitch_state_)
        depth_effort = self.depth_pid_.update(self.depth_state_)

        cmd = Wrench()
        cmd.force.x = clip_abs(self.command_.linear.x, self.max_effort_)
        cmd.force.y = clip_abs(self.command_.linear.y, self.max_effort_)
        cmd.force.z = clip_abs(self.command_.linear.z + depth_effort, self.max_effort_)
        cmd.torque.x = clip_abs(self.command_.angular.x + roll_effort, self.max_effort_)
        cmd.torque.y = clip_abs(self.command_.angular.y + pitch_effort, self.max_effort_)
        cmd.torque.z = clip_abs(self.command_.angular.z, self.max_effort_)

        self.wrench_publisher_.publish(cmd)

    def vel_callback(self, msg):
        self.command_ = msg

    def roll_callback(self, msg):
        self.roll_state_ = msg.data

    def pitch_callback(self, msg):
        self.pitch_state_ = msg.data

    def heading_callback(self, msg):
        self.heading_state_ = msg.data

    def depth_callback(self, msg):
        self.depth_state_ = msg.data


if __name__ == '__main__':
    rospy.init_node('pid_regulator')
    p = PidRegulator()

    rospy.spin()
