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

class RVO:

    all_robots_infos_dic = {} 

    def __init__(self, robot_name):
        
        # initialisations of coordinates of the robot (x,y,theta) and the velocities (v,w)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.V_x = 0
        self.W_z = 0

        # initialisations of a the middle point of the computed path of the robot - a via point
        self.middle_point_x = 0
        self.middle_point_y = 0

        # initialisations of the coordinates of the point of the front of the robot. this point belong to the line perpendicular to the robot axis
        self.xf = 0
        self.yf = 0

        # initialisations of the coordinates of the point of the back of the robot. this point belong to the line perpendicular to the robot axis
        self.xb = 0
        self.yb = 0

        # initialisation of the status of achievement of the goal
        self.there = 0

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

        #------------------------------- Publishers----------------------------------------#

        #veloctiy publisher
        self.veloctiy_pub = rospy.Publisher('/' +self.robot_name+ '/cmd_vel', Twist, queue_size=1)

        #publisher that will publish the robot's information to the common topic of type information
        self.information_pub=rospy.Publisher('/common_topic', information, queue_size=10)
    
        #-------------------------------- Subscribers--------------------------------------#

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

    #---------------------------------- Callback funtions: functions related to the topics -----------------------------------#

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
                self._plan_x.append(elem.pose.position.x)
                self._plan_y.append(elem.pose.position.y)
        
        a = int(len(self._plan_x)/2)
        b = int(len(self._plan_y)/2)
        self.middle_point_x = self._plan_x[a]
        self.middle_point_y = self._plan_y[b]

    
    # callback function of the coordinates of the robot (x,y,theta)
    def pose_callback(self, msg):

        self.x = msg.pose.pose.position.x #- 3
        self.y = msg.pose.pose.position.y #+ 1
        self.rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion ([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])
    
    # callback function of the linear and angular velocity of the robot (v,w)
    def velocity_callback(self,msg0):

        self.V_x = msg0.twist.twist.linear.x
        self.W_z = msg0.twist.twist.angular.z

    # function that publish the actual informations that are needed by other robots to the common topic
    def publish_to_information_channel(self, t):

        self.coordinate_front()
        self.coordinate_back()
        self.im_there()

        i=information()
        i.robot_name = t
        i.robot_x = self.x
        i.robot_y = self.y
        i.robot_theta = self.theta
        i.robot_velocity = self.V_x
        i.robot_xf = self.xf
        i.robot_yf = self.yf
        i.robot_state = self.there
        i.robot_xb = self.xb
        i.robot_yb = self.yb

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
        self.xf_temp = self.inf.robot_xf
        self.yf_temp = self.inf.robot_yf
        self.state_temp = self.inf.robot_state
        self.xb_temp = self.inf.robot_xb
        self.yb_temp = self.inf.robot_yb

        self.pose_updated = [self.x_temp, self.y_temp, self.theta_temp, self.velocity_temp, self.xf_temp, self.yf_temp, self.state_temp, self.xb_temp, self.yb_temp]
        self.all_robots_infos_dic.update({self.name_temp: self.pose_updated})

    # function that recieves informations about the scan by the Laserscanner
    def scan_callback(self, msg1):
        self.laser_array = msg1.ranges[90:180]
        self.min_laser = min(msg1.ranges[90:180])

        self.front_right = min(msg1.ranges[90:134])
        self.front_left = min(msg1.ranges[135:180])

    #-------------------------------------------- Velocity Obstacle functions -------------------------------------------#

    # this function calculates the region of the velocity obstacle 
    
    
    def RVO_calculation(self,v_mag):

        radius = rospy.get_param("radius") # radius variable is the radius of the circle that represent the velocity obstacle
        rr=2 # rr = 2 means that we will round all variables to only two decimals
        self.angle_to_agent = {} # angle between the robots
        self.omega = {} # angle between: the direct line between robots & the line from robot1 to the tangent of the VO
        self.VO = {} # the region of the velocity obstacle
        self.RVO = {} # the region of the reciprocal velocity obstacle. For the simplicity, in this algorithm we assume that RVO = VX 
        #self.time_to_collision = {}
        self.distance = {} # distance between the robots
        self.aanglee = 0 # Neighouring angle region initialisation
        rospy.sleep(0.01)

        self.updt_rate = 0.3 # this variable is needed in the main function to update the calculations every 0.3s when following the global path 

        self.NR = rospy.get_param("neighbour_region") #Neighbouring region in which the robots should be taken into account and thus their VO should be calculated

        for i in self.all_robots_infos_dic: # i iterate for all roboters
            if (i != self.robot_name): #  that means we should consider all the robot except the robot itself
                self.distance[i] = round(abs(sqrt(((self.all_robots_infos_dic[i][0] - self.x) ** 2) + ((self.all_robots_infos_dic[i][1] - self.y) ** 2))),rr)
                self.aanglee = round(atan2(self.all_robots_infos_dic[i][1] - self.y, self.all_robots_infos_dic[i][0] - self.x), rr)

                if self.distance[i] < self.NR:
                    self.updt_rate = 0.3
                else:
                    self.updt_rate = 0.3

                # we should calculate the velocity obstacle of the robots only when Neighbouring region and Neighbouring angle conditions are fulfilled
                if(self.distance[i] < self.NR) and (self.theta - np.pi/4 <= self.aanglee <= self.theta + np.pi/4): 
                    
                    # shifting the Velocity obstacle accordingly to the signe of the velocity of the other robot
                    # if the robot is moving forward, VO will be calculated from the front
                    # if the robot is moving backwar, VO will be calculated from the back
                    # if the robot is not moving at all, VO will be calculated from the center
                    if self.all_robots_infos_dic[i][3] > 0:
                        self.angle_to_agent[i] = round(atan2(self.all_robots_infos_dic[i][5] - self.y, self.all_robots_infos_dic[i][4] - self.x), rr)
                    elif self.all_robots_infos_dic[i][3] < 0:
                        self.angle_to_agent[i] = round(atan2(self.all_robots_infos_dic[i][8] - self.y, self.all_robots_infos_dic[i][7] - self.x), rr)
                    else:
                        self.angle_to_agent[i] = round(atan2(self.all_robots_infos_dic[i][1] - self.y, self.all_robots_infos_dic[i][0] - self.x), rr)

                    try:
                        self.omega[i] = round(asin(radius/self.distance[i]),rr)
                    except ValueError:
                        self.omega[i] = round(np.pi/3,rr) 


                    self.VO[i] = (np.asarray([self.angle_to_agent[i]- self.omega[i], self.angle_to_agent[i] + self.omega[i]])) 
                    self.keyy = i
                    self.RVO[i] = self.VO[i]
                    
    # this function controls if a heading h is inside the Velocity Obstacle
    def in_RVO(self,h): 
        for i in self.RVO:
            if (self.RVO[i][0] < h < self.RVO[i][1]):
                return True
                
            else:
                return False

    # this function returns that the collision will happen if the the actual heading theta of the robot is inside the Velocity Obstacle of the other robot
    def collision(self):
        if (self.in_RVO(self.theta) == True):
            return True
        return False

    # compute a new angular velocity/ new heading that is outside the Velocity Obstacle of other robots
    def choose_new_velocity_RVO(self):
        rr = 2
        inc = 0.01 #defining possible heading with a resolution of 0.01

        self.desired_heading = self.reference_heading # reference_heading is the last recieved heading (theta) before the activation of the collision avoidance
        self.headings_array = np.round(np.arange(- 2*np.pi, 2*np.pi, inc),rr) # array of all possible headings

        self.best_min = None 
        self.temp_array = np.array([])

        for i in self.RVO:
            self.temp_array = np.append(self.temp_array, self.RVO[i]) # itermediate variable, represents the velocity obstacle

        self._h = np.round(self.temp_array, rr) # array that contains the rounded variables of temp_array_marginals. _h is for internal use inside this function

        # delete from the headings array all positions/headings that belongs to the Velocity Obstacle
        for i in range(len(self._h)): 

            if (i%2 == 0): # if rest mta3 i/2 = 0... ki na7iw l condition hedhi tjina error ,, index 2 is out of bounds''
                k = self._h[i] + inc
                while (k< np.round (self._h[i+1], rr)):
                    self.headings_array = np.delete(self.headings_array, np.where(self.headings_array==np.round(k,rr)))
                    k+=inc
        
        # index of the heading that is outside the Velocity Obstacle but close to the desired heading
        self.idx = np.abs(self.headings_array - self.desired_heading).argmin()

        # value of the heading which the minimum index
        self.best_min = self.headings_array[(self.idx-1)%len(self.headings_array)]

        print(self.best_min)

        # controlls if the collision free velocity is really the closest to the desired heading
        # desired heading is always between -pi and pi , the velocity obstacle limits can however exceed these limits 
        # an example is when the angle_to_agent = 170° and omega = 40°
        # this way the velocity obstacle region will be between [130°,210°], in this case the upper limit exceeded pi
        # and if the desired heading is for example equal to -170°, the nearset leg will be the leg with the angle of 130°
        # but to compute a correct heading, the velocity obstacle must be in fact between [130°,-150°]
        # so the right choice here will be then the choice of the other leg (that is the angle of -150°)
        # So, if the computed velocity is not the closest, then compute whether the left of right side is the nearest to the desired heading
        # then add a small angle (0,02 rad = 1,15 degree) to the correspondant leg of the Velocity Obstacle
        # so that the computed velocity will be on the nearest side to the desired heading
        for i in self.RVO:

            if (self.angle_to_agent[self.keyy] > np.pi/2) and (self.theta < 0):
                theta_to_leg_one = abs((self.theta + 2*np.pi) - self.RVO[i][0])
                theta_to_leg_two = abs((self.theta + 2*np.pi) - self.RVO[i][1])
                print('fall 1', self.angle_to_agent)

            elif (self.angle_to_agent[self.keyy] < -np.pi/2) and (self.theta > 0):
                theta_to_leg_one = abs((self.theta - 2*np.pi) - self.RVO[i][0])
                theta_to_leg_two = abs((self.theta - 2*np.pi) - self.RVO[i][1])
                print('fall 2')
            else:
                theta_to_leg_one = abs(self.theta - self.RVO[i][0])
                theta_to_leg_two = abs(self.theta - self.RVO[i][1])
                print('fall 3')

            new_to_leg_one = abs(self.best_min - self.RVO[i][0])
            new_to_leg_two = abs(self.best_min - self.RVO[i][1])

            if np.sign(theta_to_leg_one - theta_to_leg_two) != np.sign(new_to_leg_one - new_to_leg_two):

                if theta_to_leg_one > theta_to_leg_two:
                    self.best_min = self.RVO[i][1] + 0.02
                    print('case 1')
                else:
                    self.best_min = self.RVO[i][0] + 0.02
                    print('case 2')
            else:
                continue

        betrag = abs(self.theta - self.best_min)

        if self.best_min > np.pi and self.theta <0:
            betrag = abs(self.theta - self.best_min + 2*np.pi)
        elif self.best_min < - np.pi and self.theta >0:
            betrag = abs(self.theta - self.best_min - 2*np.pi)
        
        signe= np.sign(self.best_min - self.theta)

        return betrag*signe


    # computes the coordinates of the point on the front of the robot. this point belong to the line perpendicular to the robot axis
    def coordinate_front(self):
        
        half_l = rospy.get_param("half_l")  #half the length of the robot

        offset_1 = abs(half_l * cos(self.theta))
        offset_2 = abs(half_l * sin(self.theta))

        offset_3 = abs(half_l * cos(abs(self.theta) - (np.pi/2)))
        offset_4 = abs(half_l * sin(abs(self.theta) - (np.pi/2)))

        if 0 <= self.theta <= np.pi/2:
            self.xf = self.x + offset_1
            self.yf = self.y + offset_2

        elif -np.pi/2 <= self.theta <= 0:
            self.xf = self.x + offset_1
            self.yf = self.y - offset_2

        elif np.pi/2 <= self.theta <= np.pi:
            self.xf = self.x - offset_4
            self.yf = self.y + offset_3
            
        else:
            self.xf = self.x - offset_4
            self.yf = self.y - offset_3

    
    # computes the coordinates of the point on the back of the robot. this point belong to the line perpendicular to the robot axis    
    def coordinate_back(self):
        
        half_l = rospy.get_param("half_l") #half the length of the robot

        offset_1 = abs(half_l * cos(self.theta))
        offset_2 = abs(half_l * sin(self.theta))

        offset_3 = abs(half_l * cos(abs(self.theta) - (np.pi/2)))
        offset_4 = abs(half_l * sin(abs(self.theta) - (np.pi/2)))

        if 0 <= self.theta <= np.pi/2:
            self.xb = self.x - offset_1
            self.yb = self.y - offset_2

        elif -np.pi/2 <= self.theta <= 0:
            self.xb = self.x - offset_1
            self.yb = self.y + offset_2

        elif np.pi/2 <= self.theta <= np.pi:
            self.xb = self.x + offset_4
            self.yb = self.y - offset_3
            
        else:
            self.xb = self.x + offset_4
            self.yb = self.y + offset_3


    # computed the linear velocity, which will be proportinal to the distance between the robots
    def linear_velocity(self):
        kp = rospy.get_param("kp")
        return round(self.distance[self.keyy]/kp,2)

    # the function give an information to the other robots, if this robot already reached the goal with a certain tolerance (distance_threshold)
    # so that this robot will be regarded in this case as a static obstacle
    def im_there(self):
        distance_threshold = rospy.get_param("distance_threshold")
        if self.distance_to_goal <= distance_threshold:
            self.there = 1
        else:
            self.there = 0

    #---------------------------------------------Main Function to move to the destination ---------------------------------------------#
    def moveToGoal(self,xGoal,yGoal):
        
        #define a client  to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient(""+self.robot_name+"/move_base", MoveBaseAction)

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

        #--------------------------------#
         
        self.heading = 0 # initialisation of the self.heading which will take the new collision free velocity
        self.reference_heading = 0 # initialisation of the reference heading, which represents the desired heading in the function that calculates the new velocity

        array_for_x = []
        array_for_y = []

        # if the distance between the goal and the robot is smaller than this variable, we assume that the robot reached the position
        distance_tolerance = rospy.get_param("distance_tolerance") 

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
            
            self.goal.x = self.middle_point_x 
            self.goal.y = self.middle_point_y 
            self.desired_heading = atan2(self.goal.y - self.y, self.goal.x - self.x)

            # actualisation of the distance from the robot to the goal
            self.distance_to_goal = abs(sqrt(((xGoal - self.x) ** 2) + ((yGoal - self.y) ** 2)))
            
            # computes the velocity obstacle region
            self.RVO_calculation(self.V_x)


            # we will send the collision free linear and angular velocity to the robot only when:
            # the collision is about to happen
            # AND this robot is closer to the other robot than the goal 
            # And if the other robot has not reached yet his destination (so it cannot be regarded as a static obstacle)
            if (self.collision()==True) and (self.distance[self.keyy] < self.distance_to_goal) and (self.all_robots_infos_dic[self.keyy][6] == 0): 
                self.heading = self.choose_new_velocity_RVO()

                self.vel_msg.linear.x = self.linear_velocity()
                self.vel_msg.angular.z = self.heading

                self.veloctiy_pub.publish(self.vel_msg)
                print('rotating')
                print(self.theta, self.heading, self.RVO, np.sign(self.vel_msg.angular.z))

            # we will not send any collision avoidance requests/velocities, means we will continue to follow the path with dwa local planner when:
            # collision is about to happen, BUT:
            # case 1: the robot is closer to its destination than the other robot  
            # OR
            # case 2: the robot is closer to the other robot than its own destination but the other robot has already reached his goal
            elif ((self.collision()==True) and (self.distance[self.keyy] > self.distance_to_goal)) or ((self.collision()==True) and (self.distance[self.keyy] < self.distance_to_goal) and (self.all_robots_infos_dic[self.keyy][6] == 1)): 
                ac.send_goal(goal)
                ac.wait_for_result(rospy.Duration(self.updt_rate))
                print('c2')

            else:
                
                # if the new heading after rotating is again inside a velocity obstacle of another robot, it is safe to assume that the configuarion is dense (many robots around)
                # in this case we choose again a collision free angular velocity, but with minimal linear velocity
                # the reason of the choice of a minimal linear velocity is that we need to practically turn in place to avoid a collision 
                self.RVO_calculation(self.V_x)

                if (self.in_RVO(self.theta) ==True):

                    self.heading = self.choose_new_velocity_RVO()

                    lin_vel = 0.1
                    
                    self.vel_msg.linear.x = lin_vel
                    self.vel_msg.angular.z = self.heading

                    self.veloctiy_pub.publish(self.vel_msg)
                    print('rotating again')
                    
                # exceute this command, means follow the global path with the local planner when the path is free of robots
                else:

                    ac.send_goal(goal)
                    ac.wait_for_result(rospy.Duration(self.updt_rate)) 

                    self.reference_heading = self.theta
                                   

            self.publish_to_information_channel(self.robot_name)

        #-------------- goal arrived -----------------------#
        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot reached the destination but not whithin the defined tolerane")
            #print(array_for_x)
            #print(array_for_y)
            return False
         
if __name__ == "__main__":

    robot_name = 'robot1'

    pp1 = RVO(robot_name)

    pp1.moveToGoal(3,-9) # goal coordinates

    rospy.spin()