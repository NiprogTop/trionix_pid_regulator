#!/usr/bin/env python

import rospy
import tf2_ros

from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure, Imu

from tf.transformations import euler_from_quaternion, quaternion_multiply


class Observer:
    def __init__(self):
        ns = rospy.get_param('~namespace', '')
        if len(ns) > 0: ns += '/'
        _, self.q_imu_to_base_ = self.get_transform(ns + 'imu_link', ns + 'base_link')

        self.zero_yaw_ = None
        self.zero_pressure_ = rospy.get_param('~zero_pressure', None)
        self.kpa_per_m_ = rospy.get_param('~kpa_per_m', 100.0)

        self.roll_publisher_ = rospy.Publisher('roll', Float64, queue_size=1)
        self.pitch_publisher_ = rospy.Publisher('pitch', Float64, queue_size=1)
        self.heading_publisher_ = rospy.Publisher('heading', Float64, queue_size=1)
        self.depth_publisher_ = rospy.Publisher('depth', Float64, queue_size=1)

        rospy.Subscriber('imu', Imu, self.imu_callback)
        rospy.Subscriber('pressure', FluidPressure, self.pressure_callback)

        rospy.loginfo('Observer started')

    def get_transform(self, target, source):
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)

        t = [0, 0, 0];
        q = [0, 0, 0, 1]

        try:
            transform = tfBuffer.lookup_transform(target, source,
                    rospy.Time(), rospy.Duration(1.0)).transform

            t = [transform.translation.x,
                 transform.translation.y,
                 transform.translation.z]

            q = [transform.rotation.x,
                 transform.rotation.y,
                 transform.rotation.z,
                 transform.rotation.w]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('Failed to find %s to %s transform, assign identity' % (source, target))
        finally:
            return (t, q)

    def pressure_callback(self, msg):
        if self.zero_pressure_ is None:
            self.zero_pressure_ = msg.fluid_pressure
        depth = (msg.fluid_pressure - self.zero_pressure_) / self.kpa_per_m_
        self.depth_publisher_.publish(depth)

    def imu_callback(self, msg):
        q = quaternion_multiply(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            self.q_imu_to_base_)
        roll, pitch, yaw = euler_from_quaternion(q)
        if self.zero_yaw_ is None:
            self.zero_yaw_ = yaw
        heading = yaw - self.zero_yaw_

        self.roll_publisher_.publish(roll)
        self.pitch_publisher_.publish(pitch)
        self.heading_publisher_.publish(heading)



if __name__ == '__main__':
    rospy.init_node('observer', anonymous=True)
    o = Observer()
    rospy.spin()
