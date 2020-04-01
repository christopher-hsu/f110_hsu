"""
ESE 680
RRT assignment
Author: Hongrui Zheng

This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from tf import transform_listener

# TODO: import as you need

# tuned parameters
GOAL = rospy.get_param('GOAL')
INTER_NUM = rospy.get_param('INTER_NUM')
EXTEND_MAX = rospy.get_param('EXTEND_MAX')


# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(object):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pf_topic = rospy.get_param('pose_topic')
        scan_topic = rospy.get_param('scan_topic')

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback, queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        drive_topic = rospy.get_param('drive_topic')

        rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        # class attributes
        # TODO: maybe create your occupancy grid here


        self.grid = np.zeros([100, 100]) 


        # part one:  dynamic_grid 
        # part two:  static_grid





    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        #step one: calculate the x, y position of laser scan point

        #step two: find the position of the grid cell

        #step three: update the grid (should we implement the bayesian update?)



        """
        inverse method
        r = math.sqrt((i - x)**2 + (j - y)**2)
        phi = (math.atan2(j - y, i - x) - theta + math.pi) % (2 * math.pi) - math.pi

        Find the range measurement associated with the relative bearing.
        k = np.argmin(np.abs(np.subtract(phi, meas_phi)))
        """
        return 


    def pf_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        quaternion = np.array([pose_msg.pose.orientation.x, 
                           pose_msg.pose.orientation.y, 
                           pose_msg.pose.orientation.z, 
                           pose_msg.pose.orientation.w])

        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [ pose_msg.pose.position.x, pose_msg.pose.position.y] 


        path = []


        
        tree = []   # a list include node structures
        x_start = Node()
        x_start.x = self.position[0]
        x_start.y = self.position[1]
        x_start.is_root = True
        tree.append(x_start)


        for i in range(len(samples)):
            sample_point = self.sample()  #return (x, y) 

            x_near_index = self.nearest(tree, sample_point)  #return the index in the tree
            x_near = tree[x_near_index]

            x_new = self.steer(x_near, sample_point)  # sampled_point (tuple of (float, float))
            if self.check_collision(x_near, x_rand):
                x_rand.parent = x_near_index
                tree.append(x_near)


            if self.is_goal(x_rand, goal_x, goal_y):  # if closer enough, then we can break
                path = self.find_path(tree, x_rand)
                break



       




        # use this short path and CSV file to get waypoint

       
        L = math.sqrt((waypoint[0]-self.position[0])**2 +(waypoint[1]-self.position[1])**2)
        arc = 2*goaly_veh/(L**2)
        angle = 0.3*arc
        angle = np.clip(angle, -0.4, 0.4)
       

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity 
        self.drive_pub.publish(drive_msg)


        return None

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = None
        y = None


        """
        while True:

            if self.grid[x,y] == 0
                return (x, y)
                break
        """

        # need check with occupancy grid
        return 

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        min_dis = float("inf")

        for i in range(len(tree)):
            node = tree[i]
            dis = math.sqrt( (node.x-sampled_point[0])**2 + (node.y-sampled_point[1])**2 )
            if dis < min_dis:
                min_dis = dis
                nearest_node = i

        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = Node()


        dist = math.sqrt((nearest_node.x-sampled_point[0])**2 + (nearest_node.y-sampled_point[1])**2 )

        if dist > EXTEND_MAX:
            new_node.x = nearest_node.x + (nearest_node.x-sampled_point[0])*0.1
            new_node.y = nearest_node.y + (nearest_node.y-sampled_point[1])*0.1

        else
            new_node.x = sampled_point[0]
            new_node.y = sampled_point[1]

        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
       
        path_x = np.linspace(nearest_node.x, new_node.x, num=INTER_NUM)
        path_y = np.linspace(nearest_node.y, new_node.y, num=INTER_NUM) 

        # use occupancy grid

        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """

        if math.sqrt((latest_added_node.x-goal_x)**2 + (latest_added_node.y-goal_y)**2) <= GOAL:
            return True

        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        node = latest_added_node
        
        while not node.is_root:
             path.append(node)
             index = node.parent
             node = tree[index]

        return path[::-1]



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()