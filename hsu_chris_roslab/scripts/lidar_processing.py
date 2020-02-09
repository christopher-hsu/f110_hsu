#!/usr/bin/env python
import rospy
import pdb
# from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from hsu_chris_roslab.msg import scan_range
import numpy as np
        

class LidarProcessing(object):
    """docstring for LidarProcessing"""
    def __init__(self):
        super(LidarProcessing, self).__init__()
        self.sub_ = rospy.Subscriber('scan', LaserScan, self.callback)
        self.pub_close = rospy.Publisher('closest_point', Float64, queue_size=10)
        self.pub_far = rospy.Publisher('farthest_point', Float64, queue_size=10)
        self.pub_scan = rospy.Publisher('scan_range_topic', scan_range, queue_size=10)

        self.closest_point = Float64()
        self.farthest_point = Float64()
        self.scan_range_message = scan_range()

    def callback(self,data):
        ranges = np.array(data.ranges)
        ranges = ranges[np.isfinite(ranges)]
        ranges = np.clip(ranges, data.range_min, data.range_max)
        closest_point = np.min(ranges)
        farthest_point = np.max(ranges)

        self.closest_point.data = closest_point
        self.farthest_point.data = farthest_point
        self.scan_range_message.closest_point = closest_point
        self.scan_range_message.farthest_point = farthest_point

        self.pub_close.publish(self.closest_point)
        self.pub_far.publish(self.farthest_point)
        self.pub_scan.publish(self.scan_range_message)

    
def listener():

    rospy.init_node('lidar_processing', anonymous=True)

    LidarProcessing()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()