#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

servo_offset = 0.0
#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.75 #.55
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        # PID CONTROL PARAMS
        self.kp = 0.42
        self.kd = 0.01
        self.ki = 0.0005
        self.L = 1.0
        self.forget = 0.98
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = rospy.get_time()
        self.i = 0.0
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        #drive_topic = '/nav'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    def pid_control(self, error, ranges):
        #TODO: Use kp, ki & kd to implement a PID controller
        cur_time = rospy.get_time()
        dt = cur_time - self.prev_time

        p_term = self.kp*error
        d_term = self.kd*(error-self.prev_error)/dt
        i_term = self.ki*self.integral

        angle = p_term + d_term + i_term
        angle = np.clip(angle,-0.4,0.4)
        abs_angle = np.absolute(angle)

        # Change drive speed based on steering angle
        if 0 <= abs_angle <= (5.5*np.pi/180):
            speed = 3.25
        elif (5.5*np.pi/180) < abs_angle <= (12*np.pi/180):
            speed = 2.3
        elif (12*np.pi/180) < abs_angle < (20*np.pi/180):
            speed = 1.6
        else:
            speed = 1.0

        # Publish drive instructions
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        self.prev_time = cur_time
        self.prev_error = error
        if self.i % 5 == 0:
            self.integral *= 0.03
            self.i = -1
        self.i += 1
        self.integral += error

    def followRight(self, data, ranges, angles):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # in sim angles -pi to pi (1080 idxs) (counterclockwise starting at 6oclock)
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        # p is perpendicular to the car and a is some theta away
        _pidx = int(0.25*1080)   #perpendicular idx
        _aidx = int(0.4*1080)

        pidx = self.range_preprocess(data, _pidx)
        aidx = self.range_preprocess(data, _aidx)
        theta = np.absolute(angles[pidx]-angles[aidx])
        rightDist, alpha = self.getRange(ranges[aidx],ranges[pidx],theta)
        projDist = rightDist + self.L*np.sin(alpha)

        error = DESIRED_DISTANCE_RIGHT - projDist
        return error

    def followLeft(self, data, ranges, angles):
        _pidx = int(1.0*len(ranges)-1)   #perpendicular idx
        _aidx = int(0.7 *len(ranges)-1)
        _bidx = int(0.75*len(ranges)-1)
        _cidx = int(0.82*len(ranges)-1)

        pidx = self.range_preprocess(data, _pidx)
        aidx = self.range_preprocess(data, _aidx)
        bidx = self.range_preprocess(data, _bidx)
        cidx = self.range_preprocess(data, _cidx)

        theta = angles[pidx]-angles[aidx]
        theta1 = angles[pidx]-angles[bidx]
        theta2 = angles[pidx]-angles[cidx]
        leftDist, alpha = self.getRange(ranges[aidx],ranges[pidx],theta)
        leftDist1, alpha1 = self.getRange(ranges[bidx],ranges[pidx],theta1)
        leftDist2, alpha2 = self.getRange(ranges[cidx],ranges[pidx],theta2)
        # compute mean of different readings
        mean_alpha = np.mean(np.array([alpha,alpha1,alpha2]))
        mean_left = np.mean(np.array([leftDist,leftDist1,leftDist2]))
        # project forward some distance L and compute error
        projDist = mean_left + self.L*np.sin(mean_alpha)
        error = -(DESIRED_DISTANCE_LEFT - projDist)

        return error

    def getRange(self, a_range, p_range, theta):
        x1 = a_range*np.cos(theta) - p_range
        x2 = a_range*np.sin(theta)
        alpha = np.arctan2(x1,x2)
        dist = p_range*np.cos(alpha)
        return dist, alpha

    def lidar_callback(self, data):
        angles = np.linspace(data.angle_min,data.angle_max,len(data.ranges))
        ranges = np.array(data.ranges)
        #Follow right or left wall
        # error = self.followRight(data,ranges,angles)
        error = self.followLeft(data,ranges,angles)
        #send error to pid_control
        self.pid_control(error,ranges)

    def range_preprocess(self,data,idx):
        #check for nans and out of range values
        while math.isnan(data.ranges[idx]) or data.range_min>data.ranges[idx]>data.range_max:
            idx -= 1
            #print(idx)
        return idx

def main(args):
    rospy.init_node("wall_follow_race_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
