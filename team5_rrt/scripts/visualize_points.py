#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

import os
import csv
import pdb
import numpy as np

dirname = os.path.dirname(os.path.abspath(__file__)) + '/../log/'
filename = dirname +  os.listdir(dirname)[0]

waypoints = np.loadtxt(filename, ndmin=2,delimiter=',')

# cleaning the log data to get better results
waypoints[130:430,:] = np.linspace(waypoints[130,:],waypoints[430,:],430-130)
waypoints[640:1400,:] = np.linspace(waypoints[640,:],waypoints[1400,:],1400-640)
waypoints[1640:1860,:] = np.linspace(waypoints[1640,:],waypoints[1860,:],1860-1640)
waypoints[2000:2920,:] = np.linspace(waypoints[2000,:],waypoints[2920,:],2920-2000)
waypoints = waypoints[:3000,:]



topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size="1")

rospy.init_node('view_node')

while not rospy.is_shutdown():
        markerArray = MarkerArray()
        for i in range(waypoints.shape[0]):
                if i % 2 == 0:
                        point = waypoints[i,:]
		        x = float(point[0])
		        y = float(point[1])
		        marker = Marker()
		        marker.header.frame_id = "/map"
		        marker.type = marker.SPHERE
		        marker.action = marker.ADD
		        marker.scale.x = 0.11
		        marker.scale.y = 0.11
		        marker.scale.z = 0.11
		        marker.color.a = 1.0
		        marker.color.r = 1.0
		        marker.color.g = 0.7
		        marker.color.b = 0.8
		        marker.pose.orientation.w = 1.0
		        marker.pose.position.x = x
		        marker.pose.position.y = y
		        marker.pose.position.z = 0
		        marker.id = i
		        markerArray.markers.append(marker)

	publisher.publish(markerArray)

	rospy.sleep(0.3)
