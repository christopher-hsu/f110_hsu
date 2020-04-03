#!/usr/bin/env python
import numpy as np
from numpy import linalg as LA
import math
import random
import os
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import itertools
# from tf import transform_listener

# TODO: import as you need

# tuned parameters
# GOAL = rospy.get_param('GOAL')
GOAL = 0.2
BOUNDARY = 2 #boundary buffer size
INTER_NUM = 50  #Discritization for collision checking
PATH_FILL = 5  #Upsample path with this many points
EXTEND_MAX = 0.5
MAX_ITER = 150
FORWARD = 0.65  #lookhead distance for wp
FARFORWARD = 1.4  #far lookhead distance for wp
SAMPLEINFRONT = 0.1 #how close to car to sample
SAMPLE = 1.4    #Sample range in front
SIDESAMPLE = 1.2    #Sample range to the sides
LOOKAHEAD = 0.2 #select waypoints within this buffer of FORWARD
rrtstar = True
search_range = 0.5
# INTER_NUM = rospy.get_param('INTER_NUM')
# EXTEND_MAX = rospy.get_param('EXTEND_MAX')

waypoint_path = os.path.dirname(os.path.abspath(__file__)) + '/../log/'
waypoint = np.loadtxt(waypoint_path +  os.listdir(waypoint_path)[0], ndmin=2,delimiter=',')
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
        # pf_topic = rospy.get_param('pose_topic')
        pf_topic = '/gt_pose'
        scan_topic = '/scan'
        # scan_topic = rospy.get_param('scan_topic')

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: maybe create your occupancy grid here

        map_grid_ros = rospy.wait_for_message('/map',OccupancyGrid,5.0)
        map_data = self.process_map(map_grid_ros)

        self.grid_static = map_data['map']
        
        self.origin = map_data['origin']
        self.resolution = map_data['resolution']
        self.x_offset = map_data['x_offset']
        self.y_offset = map_data['y_offset']

        self.grid_dynamic=np.zeros((self.grid_static.shape[0],self.grid_static.shape[1]))


        # TODO: create subscribers
        first_pose = rospy.wait_for_message(pf_topic,PoseStamped,2.0)
        quaternion = np.array([first_pose.pose.orientation.x, 
                           first_pose.pose.orientation.y, 
                           first_pose.pose.orientation.z, 
                           first_pose.pose.orientation.w])
        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [ first_pose.pose.position.x, first_pose.pose.position.y]

        self.pose_sub = rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback, queue_size=1)

        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        # drive_topic = rospy.get_param('drive_topic')
        drive_topic = '/drive'

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.visulize = rospy.Publisher('visualization_marker_array',MarkerArray, queue_size=1)


        # class attributes


        # part one:  dynamic_grid 
        # part two:  static_grid
        self.waypoint = waypoint[0]
        self.path_wp = self.waypoint

    @staticmethod
    def process_map(map_message):
        our_map = np.array(map_message.data).reshape(2048,2048)
        our_map = np.where(our_map==-1,1,our_map)
        our_map = np.where(our_map>0,1,our_map)
        
        our_map = our_map[850:1350,600:1400]
        origin = [map_message.info.origin.position.x,map_message.info.origin.position.y]
        resolution = map_message.info.resolution

        out = {'map':our_map,'origin':origin,'resolution':resolution,'y_offset': 850,'x_offset':600}

        return out


    def PosToMap(self,positions):
        """
            Takes numpy array of global positions and returns their indecies for
            our occupancy grid
        Args: 
            positions (numpy array shape (n*2)): global positions from ros
        Returns:
            grid_indecies (numpy array shape (2*n)): indicies of 
        """
        grid_indecies = np.zeros((2,positions.shape[0])).astype(int)
        grid_indecies[1,:] = np.clip((((positions[:,0] - self.origin[0])/self.resolution).astype(int) - self.x_offset),0,self.grid_static.shape[1]-1)
        grid_indecies[0,:] = np.clip(((((positions[:,1] - self.origin[1])/self.resolution).astype(int) - self.y_offset)),0,self.grid_static.shape[0]-1)
        return grid_indecies

    def AddBoundry(self,indicies,boundry_len = 2):
        '''
        Args: 
            indicies (numpy array shape (2*n)): indicies that you want its sourounding
        Returns:
            updated_indicies (numpy array shape (2*n)): updated indicies
        '''

        # print(range(-boundry_len,boundry_len+1))

        perms = np.array(list(itertools.product(range(-boundry_len,boundry_len+1),repeat = 2)))

        list_of_boundries = [indicies + perms[index_perm][:,np.newaxis] for index_perm in range(perms.shape[0])]

        updated_indicies = np.hstack(list_of_boundries)

        updated_indicies[0,:] = np.clip(updated_indicies[0,:],0,self.grid_static.shape[0]-1)
        updated_indicies[1,:] = np.clip(updated_indicies[1,:],0,self.grid_static.shape[1]-1)

        return updated_indicies

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here
        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:
        """
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num=ranges.shape[0])

        lidar_xy_car_frame = np.zeros((ranges.shape[0],2))

        lidar_xy_car_frame[:,0] = (ranges * np.cos(angles)) 
        lidar_xy_car_frame[:,1] = (ranges * np.sin(angles))

        R_mat = np.array([[np.cos(self.euler[2]),np.sin(self.euler[2])],
                        [-np.sin(self.euler[2]),np.cos(self.euler[2])]])

        lidar_xy_global = np.dot(lidar_xy_car_frame,R_mat)


        lidar_xy_global[:,0] += self.position[0]
        lidar_xy_global[:,1] += self.position[1]

  

        # self.visulize.publish(markerArray)
        grid_indecies = self.PosToMap(lidar_xy_global)
        # print('scan_callback: ',grid_indecies[:,1], ' position: ', self.position)
        grid_indecies_wb = self.AddBoundry(grid_indecies,boundry_len=BOUNDARY)


        self.grid_dynamic=np.zeros((self.grid_static.shape[0],self.grid_static.shape[1]))

        self.grid_dynamic[grid_indecies_wb[0,:],grid_indecies_wb[1,:]] = 1


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

 
        # Compare the waypoint with our goal point to find points in lookahead
        point_dist =  np.sqrt(np.sum(np.square(waypoint[:, 0:2]-self.position), axis=1))
        point_index_far = np.where(abs(point_dist-FARFORWARD)< LOOKAHEAD)[0]
        #Check collisions for sample waypoints
        wp_forward_far = waypoint[point_index_far,:]
        # use occupancy grid
        grid_indecies_far = self.PosToMap(wp_forward_far[:,:2]) 
        # Create mask to block out waypoints in wp_forward that are occupied

        mask_far = np.ones(len(wp_forward_far),bool)

        # for idx in range(grid_indecies_far.shape[1]):
        #     if self.grid_static[grid_indecies_far[0,idx],grid_indecies_far[1,idx]] != 0 or self.grid_dynamic[grid_indecies_far[0,idx],grid_indecies_far[1,idx]] != 0:
        #         mask_far[idx] = 0
        mask_far = np.where(self.grid_static[grid_indecies_far[0,:],grid_indecies_far[1,:]] == 0, mask_far, True) + \
                np.where(self.grid_dynamic[grid_indecies_far[0,:],grid_indecies_far[1,:]] == 0, mask_far, True)

        #Find first wp in list that is in front of the car
        for wp in wp_forward_far[mask_far]:
            l2_0 = [wp[0]-self.position[0], wp[1]-self.position[1]]
            wpx_veh = math.cos(self.euler[2])*l2_0[0] + math.sin(self.euler[2])*l2_0[1]
            wpy_veh = -math.sin(self.euler[2])*l2_0[0] + math.cos(self.euler[2])*l2_0[1]
            if abs(math.atan(wpx_veh/wpy_veh)) <  np.pi/2 and wpx_veh>0 :
                 self.waypoint = wp
                 break


        if self.check_wp_collision(self.waypoint):
            #Set waypoint as goal
            goal_x = self.waypoint[0]
            goal_y = self.waypoint[1]
            l2_0 = [goal_x-self.position[0], goal_y-self.position[1]]
            goaly_veh = -math.sin(self.euler[2])*l2_0[0] + math.cos(self.euler[2])*l2_0[1]
            #Calulate pure pursuit control
            L = math.sqrt((goal_x-self.position[0])**2 +(goal_y-self.position[1])**2)
            arc = 2*goaly_veh/(L**2)
            angle = 0.3*arc
            angle = np.clip(angle, -0.4, 0.4)

            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "drive"
            drive_msg.drive.steering_angle = angle
            drive_msg.drive.speed = 1
            self.drive_pub.publish(drive_msg)
            # print("No collisions, drive to close", goal_x, goal_y)

        # #Use RRT to go to far waypoint
        else:
            #Set waypoint as goal
            goal_x = self.waypoint[0]
            goal_y = self.waypoint[1]


            tree = []   # a list include node structures
            x_start = Node()
            x_start.x = self.position[0]
            x_start.y = self.position[1]
            x_start.cost = 0
            x_start.is_root = True
            tree.append(x_start)

            # use global frame

            for i in range(MAX_ITER):
                sample_point = self.sample()  #return (x, y) 
                #print(sample_point)
                x_near_index = self.nearest(tree, sample_point)  #return the index in the tree
                x_near = tree[x_near_index]

                x_new = self.steer(x_near, sample_point)  # sampled_point (tuple of (float, float))
          
                #check is collide between x_new and x_near
                if self.check_collision(x_near, x_new):
                    x_new.parent = x_near_index           
                    # print(x_new.parent)
                    if rrtstar:
                   
                        x_new.cost = self.cost(tree, x_new)
                        neighborhood = self.near(tree, x_new)   # it store the index
                        # neighbor_colliided
                        neighbor_colliided = []
                   
                        # print(len(neighborhood))
                        for neighbor_index in neighborhood:
                            if not self.check_collision(tree[neighbor_index], x_new):
                                neighbor_colliided.append(False)
                                continue
                            else:
                                neighbor_colliided.append(True)
                                cost = tree[neighbor_index].cost + self.line_cost(tree[neighbor_index], x_new)
                                if cost < x_new.cost:
                                    x_new.cost = cost
                                    x_new.parent = neighbor_index
                     
                        # check for other points whehter to change parents
                        for i in range(len(neighborhood)):
                            if not neighbor_colliided[i] or neighborhood[i] == x_new.parent:
                                
                                continue
                            if tree[neighborhood[i]].cost > x_new.cost + self.line_cost(x_new, tree[neighborhood[i]]):
                                tree[neighborhood[i]].parent = len(tree) #the index of x_new
                          
                    #rrtstar finish
                    tree.append(x_new)                

                    if self.is_goal(x_new, goal_x, goal_y):  # if closer enough, then we can break
                        #print("try to find!")
                        path = self.find_path(tree, x_new)
                        print("path find!")
                        break

            if path == []:

                #NEED TO FIGURE OUT WHAT TO DO IF path is not found. 2 options below are go to goal wp
                #or go to previosly found wp in path

                print("path NOT found") # And go to the orginal pure pursuit waypoint
                #Calculate L in global frame
                # L = math.sqrt((goal_x-self.position[0])**2 +(goal_y-self.position[1])**2)
                L = math.sqrt((self.path_wp[0]-self.position[0])**2 +(self.path_wp[1]-self.position[1])**2)
                #Calculate steering in local
                # l2_0 = [goal_x-self.position[0], goal_y-self.position[1]]
                l2_0 = [self.path_wp[0]-self.position[0], self.path_wp[1]-self.position[1]]
                goaly_veh = -math.sin(self.euler[2])*l2_0[0] + math.cos(self.euler[2])*l2_0[1]
                arc = 2*goaly_veh/(L**2)
                angle = 0.3*arc
                angle = np.clip(angle, -0.4, 0.4)

                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = "drive"
                drive_msg.drive.steering_angle = angle
                drive_msg.drive.speed = 1
                # self.drive_pub.publish(drive_msg)
            else:
                #Upsample path
                new_path = self.upsample_path(path)
                #Use pursuit on new set of points in path
                point_dist_path =  np.sqrt(np.sum(np.square(new_path-self.position), axis=1))
                point_index_path = np.where(abs(point_dist_path-FORWARD)< LOOKAHEAD)[0]
                #Find first wp in list that is in front of the car
                for wp in new_path[point_index_path]:
                    l2_0 = [wp[0]-self.position[0], wp[1]-self.position[1]]
                    pathx_veh = math.cos(self.euler[2])*l2_0[0] + math.sin(self.euler[2])*l2_0[1]
                    pathy_veh = -math.sin(self.euler[2])*l2_0[0] + math.cos(self.euler[2])*l2_0[1]
                    if abs(math.atan(pathx_veh/pathy_veh)) <  np.pi/2 and pathx_veh>0 :
                         # self.waypoint = wp
                         self.path_wp = wp
                         break

                #Calculate L in global frame
                L = math.sqrt((self.path_wp[0]-self.position[0])**2 +(self.path_wp[1]-self.position[1])**2)
                #Calculate steering in local
                l2_steer = [self.path_wp[0]-self.position[0], self.path_wp[1]-self.position[1]]
                goaly_veh = -math.sin(self.euler[2])*l2_steer[0] + math.cos(self.euler[2])*l2_steer[1]
                arc = 2*goaly_veh/(L**2)
                angle = 0.3*arc
                angle = np.clip(angle, -0.4, 0.4)

                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = "drive"
                drive_msg.drive.steering_angle = angle
                drive_msg.drive.speed = 1
                self.drive_pub.publish(drive_msg)

                # for i in range(new_path.shape[0]):
                markerArray = MarkerArray()
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.21
                marker.scale.y = 0.21
                marker.scale.z = 0.21
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.7
                marker.color.b = 0.8 #+ i*0.05
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = float(self.path_wp[0])
                marker.pose.position.y = float(self.path_wp[1])
                # marker.pose.position.x = float(new_path[i,0])
                # marker.pose.position.y = float(new_path[i,1])
                marker.pose.position.z = 0
                # marker.id = i
                markerArray.markers.append(marker)

                self.visulize.publish(markerArray)       

        return None

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        # R_mat = np.array([[np.cos(self.euler[2]),np.sin(self.euler[2])],
        #                 [-np.sin(self.euler[2]),np.cos(self.euler[2])]])

        while True:

            R_mat = np.array([[np.cos(self.euler[2]),np.sin(self.euler[2])],
                        [-np.sin(self.euler[2]),np.cos(self.euler[2])]])

            random_sample_car_frame = np.array([np.random.uniform(SAMPLEINFRONT,SAMPLE,1),np.random.uniform(-SIDESAMPLE,SIDESAMPLE,1)]).transpose()
            random_sample_global = np.dot(random_sample_car_frame,R_mat)

            random_sample_global[:,0] += self.position[0]

            random_sample_global[:,1] += self.position[1]

            grid_index = self.PosToMap(random_sample_global)
            #Sampling just 1 point so [,0]
            if self.grid_static[grid_index[0,0],grid_index[1,0]] == 0 and self.grid_dynamic[grid_index[0,0],grid_index[1,0]] == 0:
                return (random_sample_global[:,0] , random_sample_global[:,1])

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

        else:
            new_node.x = sampled_point[0]
            new_node.y = sampled_point[1]

        return new_node

    def upsample_path(self,path):
        if len(path) == 1:
            path_x = np.linspace(self.position[0],path[0].x, num=PATH_FILL).reshape(PATH_FILL,1)
            path_y = np.linspace(self.position[1],path[0].y, num=PATH_FILL).reshape(PATH_FILL,1)
            new_path = np.hstack((path_x,path_y))
        else: 
            for ii in range(len(path)-1):
                path_x = np.linspace(path[ii].x,path[ii+1].x, num=PATH_FILL).reshape(PATH_FILL,1)
                path_y = np.linspace(path[ii].y,path[ii+1].y, num=PATH_FILL).reshape(PATH_FILL,1)
                new = np.hstack((path_x,path_y))
                if ii == 0:
                    new_path = new
                else:
                    new_path = np.vstack((new_path,new))
        return new_path

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
       
        path_x = np.linspace(nearest_node.x, new_node.x, num=INTER_NUM).reshape(INTER_NUM,1)
        path_y = np.linspace(nearest_node.y, new_node.y, num=INTER_NUM).reshape(INTER_NUM,1)

        positions = np.hstack((path_x,path_y))
        # use occupancy grid
        grid_indecies = self.PosToMap(positions) 

        if any(self.grid_static[grid_indecies[0,:],grid_indecies[1,:]]) != 0 or any(self.grid_dynamic[grid_indecies[0,:],grid_indecies[1,:]] != 0):
            return False

        return True

    def check_wp_collision(self, waypoint):
        """
        This method should return whether the path between car and waypoint in question is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are is collision free
                              with the occupancy grid, true for free, false for collision
        """
       
        path_x = np.linspace(self.position[0], waypoint[0], num=INTER_NUM).reshape(INTER_NUM,1)
        path_y = np.linspace(self.position[1], waypoint[1], num=INTER_NUM).reshape(INTER_NUM,1)

        positions = np.hstack((path_x,path_y))
        # use occupancy grid
        grid_indecies = self.PosToMap(positions) 

        if any(self.grid_static[grid_indecies[0,:],grid_indecies[1,:]]) != 0 or any(self.grid_dynamic[grid_indecies[0,:],grid_indecies[1,:]] != 0):
            return False

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
        # print(node.parent)
        while not node.is_root:
             path.append(node)
             index = node.parent
             #print(index)
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
        return tree[node.parent].cost+self.line_cost(tree[node.parent], node)

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return math.sqrt((n1.x-n2.x)**2 + (n1.y-n2.y)**2)

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
        for i in range(len(tree)):
            dist = math.sqrt((node.x-tree[i].x)**2+(node.y-tree[i].y)**2)
            if dist < search_range and not tree[i].is_root:
                neighborhood.append(i)

        return neighborhood

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()