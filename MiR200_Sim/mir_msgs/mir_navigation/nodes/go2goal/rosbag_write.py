#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
#<div style="clear:both; margin-top:0em; margin-bottom:1em;"><a href="http://www.theconstructsim.com/ros-qa-138-how-to-set-a-sequence-of-goals-in-moveit-for-a-manipulator/" target="_blank" rel="nofollow" class="u9984449398d64f1420d1accb99a18a80"><!-- INLINE RELATED POSTS 2/3 //--><style> .u9984449398d64f1420d1accb99a18a80 { padding:0px; margin: 0; padding-top:1em!important; padding-bottom:1em!important; width:100%; display: block; font-weight:bold; background-color:#eaeaea; border:0!important; border-left:4px solid #34495E!important; box-shadow: 0 1px 2px rgba(0, 0, 0, 0.17); -moz-box-shadow: 0 1px 2px rgba(0, 0, 0, 0.17); -o-box-shadow: 0 1px 2px rgba(0, 0, 0, 0.17); -webkit-box-shadow: 0 1px 2px rgba(0, 0, 0, 0.17); text-decoration:none; } .u9984449398d64f1420d1accb99a18a80:active, .u9984449398d64f1420d1accb99a18a80:hover { opacity: 1; transition: opacity 250ms; webkit-transition: opacity 250ms; text-decoration:none; } .u9984449398d64f1420d1accb99a18a80 { transition: background-color 250ms; webkit-transition: background-color 250ms; opacity: 1; transition: opacity 250ms; webkit-transition: opacity 250ms; } .u9984449398d64f1420d1accb99a18a80 .ctaText { font-weight:bold; color:#7F8C8D; text-decoration:none; font-size: 16px; } .u9984449398d64f1420d1accb99a18a80 .postTitle { color:#000000; text-decoration: underline!important; font-size: 16px; } .u9984449398d64f1420d1accb99a18a80:hover .postTitle { text-decoration: underline!important; } </style><div style="padding-left:1em; padding-right:1em;"><span class="ctaText">Related content:</span>  <span class="postTitle">[ROS Q&A] 138 - How to set a sequence of goals in MoveIt for a manipulator?</span></div></a></div>
#def odom_callback(msg):
    # go = Odometry() is not needed
    #print ("------------------------------------------------")
    #print ("pose x = " + str(msg.pose.pose.position.x))
   #print ("pose y = " + str(msg.pose.pose.position.y))
    #print ("orientacion x = " + str(msg.pose.pose.orientation.x))
   # print ("orientacion y = " + str(msg.pose.pose.orientation.y))
    #rate.sleep()

def imu_callback(msg):
    # allez = Imu()
    print ("------------------------------------------------")
    print ("veloc angular z = " + str(msg.angular_velocity.z))
    print ("veloc angular y = " + str(msg.angular_velocity.y))
    print ("aceleracion linear x = " + str(msg.linear_acceleration.x))
    print ("aceleracion linear y = " + str(msg.linear_acceleration.y))
    #rate.sleep()

def twist (msg):
    # move = Twist()
    print ("velocidad linear x = " + str(move.linear.x))
    print ("velocidad angular z = " + str (move.angular.z))
    rate.sleep()
    #sub=rospy.Subscriber('cmd_vel', Twist, twist)

rospy.init_node('sphero_monitor') # the original name sphero might be the same as other node.
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #topic publisher that allows you to move the sphero
#sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, odom_callback) # the original name odom might be the same as other function.
sub_imu = rospy.Subscriber('/sphero/imu/data3', Imu, imu_callback)
#rate = rospy.Rate(0.5)

#while not rospy.is_shutdown():
    #move = Twist()
    #move.linear.x = 0.1 # m/s. The original value 2 is too large
    #move.angular.z= 0.5 # rad/s
    #pub.publish(move)
    #rate.sleep() # Instead of using rospy.spin(), we should use rate.sleep because we are in a loop



