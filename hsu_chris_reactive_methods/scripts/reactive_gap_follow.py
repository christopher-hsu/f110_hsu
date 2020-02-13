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
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    
    @staticmethod
    def running_mean(x,N):
        cumsum = np.cumsum(np.insert(x,0,0))
        return (cumsum[N:]- cumsum[:-N]) / float(N)

    def preprocess_lidar(self, data, ranges, angles):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        N = 5 #running mean window
        valid = np.arange(270,810)  #real lidar is from -1.57 to 1.57
        proc_ranges = ranges[valid]
        proc_angles = angles[valid]
        proc_ranges = self.running_mean(proc_ranges,N)
 
        return proc_ranges, proc_angles

    def bubble_lidar(self, ranges, angles):
        radius = 0.2
        #Find closest point (point of interest) to LiDAR
        poi_val = np.min(ranges)
        poi_idx = np.argmin(ranges)
        #Eliminate all points inside 'bubble' (set them to zero) 
        if poi_val == 0:
            alpha = np.arcsin(0)
        elif poi_val > radius:
            alpha = np.arcsin(radius/poi_val)
        else:
            alpha = np.arcsin(1)
        bub_idx_r = np.argmin(np.abs(angles-(angles[poi_idx]-alpha)))
        bub_idx_l = np.argmin(np.abs(angles-(angles[poi_idx]+alpha)))
        bubble = np.arange(bub_idx_r,bub_idx_l)

        bubble_ = np.clip(bubble,0,len(ranges)-1)
        ranges[bubble_] = 0.0

        return ranges, poi_val, poi_idx

    def find_max_gap(self, ranges):
        """ Return the start index & end index of the max gap in ranges
        """
        ranges[ranges!=0] = 1.0
        ranges1 = np.hstack((np.copy(ranges),0))
        ranges2 = np.hstack((0,np.copy(ranges)))
        check = ranges1 - ranges2
        #start and end indices of gaps
        start = np.where(check==1)[0]
        end = np.where(check==-1)[0]-1
        #check which gap is larger
        big_gap_idx = np.argmax(end-start)
        return start[big_gap_idx], end[big_gap_idx]
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        # best_idx = np.argmax(ranges[start_i:end_i])
        best_idx = int(np.mean([start_i,end_i]))
        return best_idx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        angles = np.linspace(data.angle_min,data.angle_max,len(data.ranges))
        ranges = np.array(data.ranges)
        proc_ranges, proc_angles = self.preprocess_lidar(data, ranges, angles)
        # Make a copy for analysis
        virt_ranges = np.copy(proc_ranges)
        # Zero out points in bubble
        virt_ranges, poi_val, poi_idx = self.bubble_lidar(virt_ranges, proc_angles)
        #Find max length gap in virtual range
        start, end = self.find_max_gap(virt_ranges)
        #Find the best point in the gap of processed ranges
        best = self.find_best_point(start, end, proc_ranges)
        #Associated angle with best point
        angle = proc_angles[best]
        abs_angle = np.absolute(angle)

        if 0 <= abs_angle <= (10*np.pi/180):
            speed = 1.8
        elif (10*np.pi/180) < abs_angle < (20*np.pi/180):
            speed = 1.2
        else:
            speed = 1.0

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
