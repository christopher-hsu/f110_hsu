#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import std_msgs.msg

class Analysis(object):


    def __init__(self):
        self.odom_x = 0
        self.odom_y  = 0
        self.match_x = 0

        self.match_y = 0
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber("/scan_match_location", PoseStamped, self.match_callback, queue_size=10)
        
        self.position = rospy.Publisher('/position_error', std_msgs.msg.Float64, queue_size = 10)


    def odom_callback(self, odom_msg):
        self.odom_x = odom_msg.pose.pose.position.x
        self.odom_y = odom_msg.pose.pose.position.y


    def match_callback(self, match_msg):
        self.match_x = match_msg.pose.position.x
        self.match_y = match_msg.pose.position.y
        MSE = (self.odom_x-self.match_x)*(self.odom_x-self.match_x) + (self.odom_y-self.match_y)*(self.odom_y-self.match_y) 
        self.position.publish(MSE)       


def main():
    rospy.init_node('yuweiwu_scan_matching_lab')
    sn = Analysis()
    rospy.spin()
    
if __name__ == '__main__':
    main()
