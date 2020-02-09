#!/usr/bin/env python
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
        

def callback(data):
    # ranges = np.array(data.ranges)
    # ranges = ranges[np.isfinite(ranges)]
    # ranges = np.clip(ranges, data.range_min, data.range_max)


    # closest_point = np.min(ranges)
    # farthest_point = np.max(ranges)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    # return closest_point, farthest_point

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("farthest_point", Float64, callback)
    # import pdb;pdb.set_trace()
    # pub = rospy.Publisher('closest_point', Float64, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()