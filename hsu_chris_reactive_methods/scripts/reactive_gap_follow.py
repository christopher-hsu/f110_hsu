#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        self.bubble = 4
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    
    def preprocess_lidar(self, data, ranges, angles):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        valid = np.arange(135,945)
        proc_ranges = ranges[valid]
        proc_angles = angles[valid]
        # ranges[np.isnan(ranges)] = 0.0
        # proc_ranges = np.clip(ranges, data.range_min, data.range_max)

        return proc_ranges, proc_angles

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        return None

    def bubble_lidar(self, ranges):
        val_bubble = 0.2
        radius = 100
        #Find closest point (point of interest) to LiDAR
        poi = np.min(ranges)
        #Eliminate all points inside 'bubble' (set them to zero) 
        ranges[ranges < poi+val_bubble] = 0.0
        close = np.where(ranges == 0)[0]
        # print(close)
        bubble = np.arange(np.min(close)-radius, np.max(close)+radius)
        bubble_ = np.clip(bubble,0,len(ranges)-1)
        ranges[bubble_] = 0.0
        # bubble0 = close - 2
        # bubble1 = close - 1
        # bubble2 = close + 1
        # bubble3 = close + 2
        # ranges[bubble0] = 0
        # ranges[bubble1] = 0
        # ranges[bubble2] = 0
        # ranges[bubble3] = 0
        # print(np.where(ranges==0)[0])

        return ranges, poi

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        angles = np.linspace(data.angle_min,data.angle_max,len(data.ranges))
        ranges = np.array(data.ranges)
        proc_ranges, proc_angles = self.preprocess_lidar(data, ranges, angles)
        # Zero out points in bubble
        virt_ranges, poi = self.bubble_lidar(proc_ranges)
        # print(virt_ranges)
        #Find max length gap 
        #Find the best point in the gap 
        best = np.argmax(virt_ranges)

        angle = proc_angles[best]
        abs_angle = np.absolute(angle)
        # print(best)

        if 0 <= abs_angle <= (10*np.pi/180):
            speed = 1.8
            # if np.max(ranges[432:648]) > 5:
                # speed = 2.5
        elif (10*np.pi/180) < abs_angle < (20*np.pi/180):
            speed = 1.2
        else:
            speed = 0.5

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("gap_follow_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
