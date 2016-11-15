#!/usr/bin/env python

import rospy, tf
import A_Star
import time
import math
from tf.transformations import euler_from_quaternion
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid



def MapCallback(occupancy):
	#OccupancyGrid map 
	global mapSet, occupancyGrid
	mapSet = 1
	occupancyGrid = occupancy

if __name__ == '__main__':
	#initialize globals
	global mapSet, occupancyGrid
	mapSet = 0
	occupancyGrid = None
	rospy.init_node('tstephen_Lab_3_node')
	rospy.Subscriber('map', OccupancyGrid, MapCallback)
	odom_list = tf.TransformListener()
	rospy.sleep(2)
	print "Starting Lab 3"

	
	#inititalized mapSet = 0
	while not mapSet:
		sleep(.1)
	start_pos = Point()
	goal_pos = Point()
	#Error placed goal in wall (2,2) goal (25,30)
	start_pos.x = 7
	start_pos.y = 1
	start_pos.z = 0
	goal_pos.x = 20
	goal_pos.y = 20
	goal_pos.z = 0

	newPath = A_Star.GetPath(occupancyGrid, start_pos, goal_pos)
	print "Calculating best path"
	A_Star.Waypoints(newPath)


	print "Complete"
