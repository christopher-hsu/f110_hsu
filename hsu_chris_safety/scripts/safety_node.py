#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import numpy as np
# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub_brake = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        self.pub_brake_bool = rospy.Publisher('/brake_bool', Bool, queue_size=10)

        self.brake_msg = AckermannDriveStamped()
        self.brake_bool_msg = Bool()

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        thetas = np.linspace(scan_msg.angle_min,scan_msg.angle_max,len(scan_msg.ranges))
        range_rate = self.speed*np.cos(thetas)
        range_rate[range_rate<=0] = 0.00001
        ranges = self.range_preprocess(scan_msg)
        TTC = np.divide(ranges,range_rate)
        # TODO: publish brake message and publish controller bool
        if np.min(TTC) < .33:
            self.brake_bool_msg.data = True
            self.brake_msg.drive.speed = 0.0
            self.pub_brake_bool.publish(self.brake_bool_msg)
            self.pub_brake.publish(self.brake_msg)
        else:
            self.brake_bool_msg.data = False
            self.pub_brake_bool.publish(self.brake_bool_msg)

    def range_preprocess(self,scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges = np.clip(ranges, scan_msg.range_min, scan_msg.range_max)
        return ranges

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()