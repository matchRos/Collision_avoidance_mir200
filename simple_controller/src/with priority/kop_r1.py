#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped, Quaternion, PoseStamped, Pose, TwistWithCovariance
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi, cos, sin, asin, atan, radians, degrees, tan
from sensor_msgs.msg import LaserScan, Imu
from simple_controller.msg import information
from random import randrange
import importlib
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import sys
import itertools


# this code can be considered as a draft because it is not the main Method used in the thesis
# this code was made and intended for two robots
# it was made to prove that a navigation with an exchange of paths between robots is not very efficient
# it consumes much more computing resource and time than the other code

class RVO:

    all_robots_infos_dic = {} 

    def __init__(self, robot_name):

        # initialisations of coordinates of the robot (x,y,theta) and the velocities (v,w)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.V_x = 0
        self.W_z = 0

        #initialisations of coordinates of the collision point
        self.x_collide = 0
        self.y_collide = 0

        # initialisations of a the middle point of the computed path of the robot - a via point
        self.middle_point_x = 0
        self.middle_point_y = 0

        # initialisation of the x and y coordinates of each point of the computed path of the robot
        self._plan_x = []
        self._plan_y = []

        # create a node with the name of the robot
        self.robot_name = robot_name
        print(self.robot_name)
        rospy.init_node(self.robot_name)

        # velocity message of type Twist()
        # information message of type information()
        # the goal coordinates of type Point()
        self.vel_msg = Twist()
        self.inf = information()
        self.goal = Point()
        
        #initialisation of the information whether the robot at the end achieved the goal within the defined tolerance range
        # this information is not relevant to the computation, but only to the user at the end of the navigation
        self.goalReached = False

        #------------------------------- Publishers------------------------------------#

        #veloctiy publisher
        self.veloctiy_pub = rospy.Publisher('/' +self.robot_name+ '/cmd_vel', Twist, queue_size=1)

        #publisher that will publish the robot's informations to the common topic of type information
        self.information_pub=rospy.Publisher('/common_topic', information, queue_size=10)
    
        #-------------------------------- Subscribers--------------------------------#

        # self.velocity_callback is called when a message of type Odometry is received, the relevant informations here are the linear and angular velocities
        self.pose_sub=rospy.Subscriber('/' +self.robot_name+ '/mobile_base_controller/odom', Odometry, self.velocity_callback)

        # self.pose_callback is called when a message of type PoseWithCovarianceStamped is received, the relevant informations here are coordinates of the robot
        self.pose_sub=rospy.Subscriber('/' +self.robot_name+ '/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # self.path_callback is called when the path is computed and available
        self.path_sub=rospy.Subscriber('/' +self.robot_name+ '/move_base_node/SBPLLatticePlanner/plan', Path, self.path_callback)

        # self.recieve_from_information_channel is called whenever a message of type information is recieved from the common topic
        rospy.Subscriber('/common_topic', information, self.recieve_from_information_channel)

        # self.scan_callback is called when a message of type LaserScan is recieved
        self.scan_sub=rospy.Subscriber('/' +self.robot_name+ '/scan', LaserScan, self.scan_callback)

    #---------------------------------- Callback funtions ----------------------------------------------------------#

    # callback function of the path. it serves to fill the x and y coordinates of each point of the computed path of the robot in two arrays
    # dist is the distance between each point of the computed path and the robot  
    # the points will only be saved in the array if the distance between them and the robot exceed the minimum distance dist
    # the path is computed beginning from the center of the robot
    # the 0.2m serves to save some space, so only point that are 0.2m away from the robot are saved
    # (self.middle_point_x, self.middle_point_y) is the coordinate of the middle point of the computed path of the robot - a via point

    def path_callback(self,msg2):

        for elem in msg2.poses:
            dist = round(abs(sqrt(((elem.pose.position.x - self.x) ** 2) + ((elem.pose.position.y - self.y) ** 2))),2)
            if dist >= 0.2:
                self._plan_x.append(round(elem.pose.position.x,2))
                self._plan_y.append(round(elem.pose.position.y,2))
        
        a = int(len(self._plan_x)/2)
        b = int(len(self._plan_y)/2)
        self.middle_point_x = self._plan_x[a]
        self.middle_point_y = self._plan_y[b]

    # callback function of the coordinates of the robot (x,y,theta)
    def pose_callback(self, msg):

        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y 
        self.rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion ([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])
    
    # callback function of the linear and angular velocity of the robot (v,w)
    def velocity_callback(self,msg0):

        self.V_x = msg0.twist.twist.linear.x
        self.W_z = msg0.twist.twist.angular.z

    # function that publish the actual informations that are needed by other robots to the common topic
    def publish_to_information_channel(self, t):

        i=information()
        i.robot_name = t
        i.robot_x = self.x
        i.robot_y = self.y
        i.robot_theta = self.theta
        i.robot_velocity = self.V_x
        i.robot_plan_x = self._plan_x
        i.robot_plan_y = self._plan_y
        i.robot_time = self.time_to_reach_inters()

        self.information_pub.publish(i)

    # function that recieves the informations of other robots from the common topic
    # at the end we will have a dictionary, its key is the name of the robot, its values are the recieved informations    
    def recieve_from_information_channel(self, msg):
        self.inf = msg
        self.name_temp = self.inf.robot_name
        self.x_temp = self.inf.robot_x
        self.y_temp = self.inf.robot_y
        self.theta_temp = self.inf.robot_theta
        self.velocity_temp = self.inf.robot_velocity
        self.plan_x_temp = self.inf.robot_plan_x
        self.plan_y_temp = self.inf.robot_plan_y
        self.time_temp = self.inf.robot_time

        self.pose_updated = [self.x_temp, self.y_temp, self.theta_temp, self.velocity_temp, self.plan_x_temp, self.plan_y_temp, self.time_temp]
        self.all_robots_infos_dic.update({self.name_temp: self.pose_updated})

    # function that recieves informations about the scan by the Laserscanner
    def scan_callback(self, msg1):
        self.laser_array = msg1.ranges[90:180]
        self.min_laser = min(msg1.ranges[90:180])

        self.front_right = min(msg1.ranges[90:134])
        self.front_left = min(msg1.ranges[135:180])

    #-------------------------------------------------NEW--------------------------------------------------------------#


    # this function searches for the minimum distance between two paths, which are the paths of two robots
    # it computes the distance between all point from path 1 and all points from path 2
    # it then chooses the minimum distane
    # when the minimum distance is smaller than a critic distance d_kritisch,
    # then the paths have either an intersection point 
    # or
    # there is a minimum distance between the paths, where the robots can touch it other from the side, if they reach that point at the same time

    def intersec_point(self):

        d_kritisch = 1

        list_of_dist=[]
        list_of_x=[]
        list_of_y=[]
        min_value = 0
        
        for i in self.all_robots_infos_dic:  
            if (i != self.robot_name):

                # for (q,p) in zip (self.all_robots_infos_dic[i][4], self.all_robots_infos_dic[i][5]):
                #     for (j,k) in zip(self._plan_x, self._plan_y):

                for (j,k) in zip(self._plan_x, self._plan_y):
                    for (q,p) in zip (self.all_robots_infos_dic[i][4], self.all_robots_infos_dic[i][5]):

                        d_n = round(abs(sqrt(((k - p)**2) + ((j - q)**2))),2)
                        
                        list_of_dist.append(d_n)   
                        list_of_x.append(j)
                        list_of_y.append(k)

                    min_val = min(list_of_dist)  
                    min_index = list_of_dist.index(min_val)
                    
                    a = list_of_dist[min_index]
                    b = list_of_x[min_index]
                    c = list_of_y[min_index]

                    del list_of_dist[:]
                    del list_of_x[:]
                    del list_of_y[:]

                    list_of_dist.append(a)
                    list_of_x.append(b)
                    list_of_y.append(c)

                min_value = min(list_of_dist)
                min_ind = list_of_dist.index(min_value)

                self.x_collide = list_of_x[min_ind]
                self.y_collide = list_of_y[min_ind]

        if min_value < d_kritisch:
            return True
        else:
            return False


    # this function computes the distance between the intersection point of the path and the robot
    # however the distance is not computed with the euclidean distance
    # a path is an array of points, so we take the euclidean distance between each two points in that array from the actual position of the robot till the intersection point
    # these points are very close to each other, so the sum of these small distances will give the actual distance along the path

    def length_of_path(self):
        
        if (self.intersec_point() == True):
            summe = 0
            list_distances = []
            arr_plan_x =[]
            arr_plan_y =[]

            for (f,g) in zip(self._plan_x,self._plan_y):
                if (f == self.x_collide) and (g == self.y_collide):
                    break
                else:
                    arr_plan_x.append(f)
                    arr_plan_y.append(g)
            
            arr_plan_x.reverse()
            arr_plan_y.reverse()

            n = len(arr_plan_x) -1 
            m = len(arr_plan_y) -1
            for (i,j) in zip (range(n), range(m)): 
                d = round(abs(sqrt(((arr_plan_y[j+1]- arr_plan_y[j])**2) + ((arr_plan_x[i+1] - arr_plan_x[i])**2))),2)
                list_distances.append(d)
            
            for p in list_distances:
                summe = summe + p

            return round(summe,2)
        
        else:
            return False


    # when the distance from the robot to the intersection point is computed , we calculate the time needed to reach that point
    def time_to_reach_inters(self):

        if (self.length_of_path() != False):
            abstand = self.length_of_path()
            t = abstand / self.V_x
            return round(t,2)
        else:
            return False

    # after computing the time needed to reach the intersection point, we verify if the robots will reach this point at the same time 
    # so we calculate the the difference between the time needed to reach this point for every robot
    # if the difference is smaller than a critic time t_kritisch, then a collision will happen 
    def kollision(self):
        t_kritisch = 3
        for i in self.all_robots_infos_dic:  
            if (i != self.robot_name):

                if (self.time_to_reach_inters() != False):
                    if abs(self.time_to_reach_inters() - self.all_robots_infos_dic[i][6]) < t_kritisch:
                        return True
                    else:
                        return False
                else:
                    return False

    # this function assign priority to each robot
    # the robot that is closer to the intersection point has higher priority
    def priority(self):
        for i in self.all_robots_infos_dic:  
            if (i != self.robot_name):

                if self.time_to_reach_inters() < self.all_robots_infos_dic[i][6]:
                    return True
                else:
                    return False

    #---------------------------------------------move to the destination ---------------------------------------------#
    
    def moveToGoal(self,xGoal,yGoal):
        
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("robot1/move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")

        # creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal with the coordinates (xGoal,yGoal)
        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        start = rospy.get_time()  #get the current time

        rospy.loginfo("Sending goal location ...")

        #---------------------------------------------#
        array_for_x = []
        array_for_y = []

        array_for_v = []
        array_for_w = []

        # if the distance between the goal and the robot is smaller than this variable, we assume that the robot reached the position
        distance_tolerance = 0.5

        self.goal.x = self.middle_point_x
        self.goal.y = self.middle_point_y

        self.desired_heading = atan2(self.goal.y - self.y, self.goal.x - self.x)
        self.distance_to_goal = abs(sqrt(((xGoal - self.x) ** 2) + ((yGoal - self.y) ** 2)))

        # returns to the desired heading, so that the global planner will take in account the actual heading and computes the path accordingly
        # but thats only valide if there is no obstacles after turning
        self.vel_msg.linear.x = 0.3
        self.vel_msg.angular.z = 0.5 * (self.desired_heading-self.theta)
                
        self.veloctiy_pub.publish(self.vel_msg)

        while (self.distance_to_goal > distance_tolerance):
            array_for_x.append(self.x)
            array_for_y.append(self.y)

            array_for_v.append(self.V_x)
            array_for_w.append(self.W_z)

            self.goal.x = self.middle_point_x
            self.goal.y = self.middle_point_y 

            self.desired_heading = atan2(self.goal.y - self.y, self.goal.x - self.x)
            
            # actualisation of the distance from the robot to the goal
            self.distance_to_goal = abs(sqrt(((xGoal - self.x) ** 2) + ((yGoal - self.y) ** 2)))
            
            # computes the intersection point, if available
            self.intersec_point()

            # when a collision would happen, the robot with a lesser priority should reduce its velocity to 70%
            if (self.kollision()==True) and (self.priority() == False): 
                
                self.vel_msg.linear.x = 0.7 * self.V_x
                self.vel_msg.angular.z = 0.7 * self.W_z
                self.veloctiy_pub.publish(self.vel_msg)
                print('velocity will be adjusted', self.V_x, self.vel_msg.linear.x, self.W_z, self.vel_msg.angular.z)

            # this command tells the robot to continue with the dwa along the global plan if no collision is about to happen or when the robot has high priority
            else:
                
                ac.send_goal(goal)
                ac.wait_for_result(rospy.Duration(0.3)) 

            self.publish_to_information_channel(self.robot_name)

        #---------------------------- goal arrived -----------------------------#
        if (ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot reached the destination but not whithin the defined tolerane")
            return False

if __name__ == "__main__":

    robot_name = 'robot1'

    pp1 = RVO(robot_name)

    pp1.moveToGoal(3,-9) # coordinates of the goal

    rospy.spin()