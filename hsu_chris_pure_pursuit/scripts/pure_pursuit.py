#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import tf2_ros
# position_x, position_y, euler_yaw, speed
import os
import numpy as np
import tf
import math


path = os.path.dirname(os.path.abspath(__file__)) + '/../logs/'
waypoint = np.loadtxt(path +  os.listdir(path)[0], ndmin=2,delimiter=',')
FORWARD = 0.7


class PurePursuit(object):
    """
    The" class that handles pure pursuit.
    """
    def __init__(self):
        self.waypoint  = waypoint[0]
        self.index = 0
        pose_topic = '/gt_pose'
        drive_topic = '/drive'
        #'/nav' is for navigation, and /drive directly control it
        self.pose_sub = rospy.Subscriber( pose_topic, PoseStamped, self.pose_callback, queue_size=1)

        self.drive_pub = rospy.Publisher( drive_topic, AckermannDriveStamped, queue_size=1)
        
        
 
    

    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture
        quaternion = np.array([pose_msg.pose.orientation.x, 
                           pose_msg.pose.orientation.y, 
                           pose_msg.pose.orientation.z, 
                           pose_msg.pose.orientation.w])

        euler = tf.transformations.euler_from_quaternion(quaternion)
        position = [ pose_msg.pose.position.x, pose_msg.pose.position.y] 
       

        point_dist =  np.sqrt(np.sum(np.square(waypoint[:, 0:2]-position), axis=1))
        point_index = np.where(abs(point_dist-FORWARD)< 0.2)[0]
        #print(point_index)
        for index in point_index:
            l2_0 = [waypoint[index, 0]-position[0], waypoint[index,1]-position[1]]
            goalx_veh = math.cos(euler[2])*l2_0[0] + math.sin(euler[2])*l2_0[1]
            goaly_veh = -math.sin(euler[2])*l2_0[0] + math.cos(euler[2])*l2_0[1]
            #print(goalx_veh, goaly_veh)
            #print(abs(math.atan(goalx_veh/goaly_veh)))
            if abs(math.atan(goalx_veh/goaly_veh)) <  np.pi/2 and goalx_veh>0 :
                 self.waypoint = waypoint[index] 
                 #print("point find")
                 break


        #print(position)


        #print(self.waypoint)
        
        # we need to find the closed point in our orientation range.
        
        # TODO: transform goal point to vehicle frame of reference
        #  tfBuffer = tf2_ros.Buffer()
        #  listener = tf2_ros.TransformListener(tfBuffer)
        # trans = tfBuffer.lookup_transform("laser_model", "base_link", rospy.Time.now()
        # listener = tf.TransformListener()
        #(trans,rot) = listener.lookupTransform("base_link","laser_model", rospy.Time(0))

        # 2d transform matrix
        l2_0 = [self.waypoint[0]-position[0], self.waypoint[1]-position[1]]
        goalx_veh = math.cos(euler[2])*l2_0[0] + math.sin(euler[2])*l2_0[1]
        goaly_veh = -math.sin(euler[2])*l2_0[0] + math.cos(euler[2])*l2_0[1]  

       

        # TODO: calculate curvature/steering angle
        L = math.sqrt((self.waypoint[0]-position[0])**2 +  (self.waypoint[1]-position[1])**2 )

        #print(L, goaly_veh )
        arc = 2*goaly_veh/(L**2)
        #print(arc)
        angle = 0.3*arc
        angle = np.clip(angle, -0.4, 0.4)
        velocity = self.select_velocity(angle)
        #print(angle, velocity)
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity 
        self.drive_pub.publish(drive_msg)
    

    def select_velocity(self, angle):
        if abs(angle) <= 5*math.pi/180:
            velocity  = 4
        elif abs(angle) <= 10*math.pi/180:
            velocity  = 3
        elif abs(angle) <= 15*math.pi/180:
            velocity = 2.5
        elif abs(angle) <= 20*math.pi/180:
            velocity = 2
        else:
            velocity = 1.5
        return velocity


def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()
