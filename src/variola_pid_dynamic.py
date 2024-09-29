#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Wrench

from pid_dynamic import PID


class PidRegulator:
    enabled_ = False
    command_ = Twist()

    def __init__(self):
        self.max_effort_ = abs(rospy.get_param("~max_effort", 1.0))

        self.pids_ = {'roll': PID('roll', enabled=True),
                      'pitch': PID('pitch', enabled=True),
                      'heading': PID('heading', enabled=False),
                      'depth': PID('depth', enabled=False)}

        for name in self.pids_.keys():
            rospy.Subscriber(name, Float64, self.state_callback, name)

        self.wrench_publisher_ = rospy.Publisher('command', Wrench, queue_size=1)
        self.switch_service_ = rospy.Service('~switch', SetBool, self.handle_switch)
        self.update_timer_ = rospy.Timer(rospy.Duration(0.1), self.update)

        rospy.Subscriber('teleop_command', Twist, self.teleop_callback)
        rospy.loginfo('Pid started')

    def handle_switch(self, req):
        self.enabled_ = req.data
        resp = SetBoolResponse(
            success=True,
            message=f"pid {('disabled', 'enabled')[self.enabled_]}")
        self.wrench_publisher_.publish(Wrench())
        return resp

    def clamp(self, v):
        return min(max(v, -self.max_effort_), self.max_effort_)

    def update(self, _):
        cmd = Wrench()
        if self.enabled_:
            cmd.force.x = self.clamp(self.command_.linear.x)
            cmd.force.y = self.clamp(self.command_.linear.y)
            cmd.force.z = self.clamp(self.command_.linear.z + self.pids_['depth'].get_effort())
            cmd.torque.x = self.clamp(self.command_.angular.x + self.pids_['roll'].get_effort())
            cmd.torque.y = self.clamp(self.command_.angular.y + self.pids_['pitch'].get_effort())
            cmd.torque.z = self.clamp(self.command_.angular.z + self.pids_['heading'].get_effort())
        self.wrench_publisher_.publish(cmd)

    def teleop_callback(self, msg):
        self.command_ = msg

    def state_callback(self, msg, pid_name):
        self.pids_[pid_name].update(msg.data)


if __name__ == '__main__':
    rospy.init_node('pid_regulator')
    p = PidRegulator()
    rospy.spin()
