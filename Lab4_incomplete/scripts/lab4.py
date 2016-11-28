#!/usr/bin/env python

import rospy, tf

import A_Star, ObstacleExpan, Waypoint_math

from tf.transformations import euler_from_quaternion

from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

import time
import math

#def MapCallback(occupancy):
	#OccupancyGrid map 
	#global mapSet, occupancyGrid
	#mapSet = 1
	#occupancyGrid = occupancy

def MapCallback(occupancy):
	global map_ready, occupancyGrid, stop
	print "looking for a map"
	# need the origin and resolution
	map_origin = occupancy.info.origin.position
	grid_res = occupancy.info.resolution
	grid_origin = Point(map_origin.x + grid_res/2 , map_origin.y + grid_res/2 , 0)
	# 1 is True
	map_ready = 1
	#stop = 1
	occupancyGrid = occupancy

def GoalCallback(goal_Point):
	global goal_ready, goal
	# 1 is True
	goal_ready = 1
	goal.x = goal_Point.pose.position.x
	goal.y = goal_Point.pose.position.y
	goal.z = 0
	print "printing goal"
	print goal

#Odometry Callback function

def timerCallback(event):
	global x, y, theta
	
	try:
		(trans,orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
		quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
		roll, pitch, yaw = euler_from_quaternion(quaternion)
		x = trans[0]
		y = trans[1]
		theta = yaw
		print x, y, theta
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass

def OdometryCallback(msg):
	#Current x, y, and theta
	#found in Lab 2 powerpoint
	global x, y, theta
	x_pos = msg.pose.pose.position.x
	y_pos = msg.pose.pose.position.y
	orientation = msg.pose.pose.orientation
	quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
	roll, pitch, yaw = euler_from_quaternion(quaternion)

	x = x_pos
	y = y_pos
	theta = yaw


#def publishTwist(linearVelocity, angularVelocity): #(powerpoint)
    #global pub
    #msg = Twist()
    #msg.linear.x = linearVelocity
    #msg.angular.z = angularVelocity
    #pub.publish(msg)

def PublishTwist(linearVelocity, angularVelocity):
	msg = Twist()
	msg.linear.x = linearVelocity
	msg.linear.y = 0
	msg.linear.z = 0
	msg.angular.x = 0
	msg.angular.y = 0
	msg.angular.z = angularVelocity
	twist_pub.publish(msg)

#Drive straight function
def DriveStraight(speed, distance):


    global pose

    initialX = pose.pose.position.x
    initialY = pose.pose.position.y
    atTarget = False
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.pose.position.x
        currentY = pose.pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        if (currentDistance >= distance):
            atTarget = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)
            rospy.sleep(0.15)

def Rotate(angle_of_rot):

	global theta

	tol = math.pi / 36
	closer_tol = math.pi / 256
	angle_goal = (theta + angle_of_rot) 

	if (angle_goal > math.pi):
		angle_goal -= (math.pi * 2)
	elif (angle_goal < (-1 * math.pi)):
		angle_goal += (math.pi * 2)

	while (theta < angle_goal - tol or theta > angle_goal + tol):
		if (angle_of_rot < 0):
			PublishTwist(0, math.pi / -4)
		else:
			PublishTwist(0, math.pi / 4)
		print theta

		time.sleep(.1)

		print "I am at", theta
		print "goalRotate", angle_goal

	while (theta < angle_goal - closer_tol or theta > angle_goal + closer_tol):
		if (angle_of_rot < 0):
			PublishTwist(0, math.pi / -12)
			if (theta < angle_goal + closer_tol):
				break
		else:
			PublishTwist(0, math.pi / 12)
			if (theta > angle_goal - closer_tol):
				break
		print theta

		time.sleep(.05)
		print "I am at", theta
		print "goalRotate precision", angle_goal 

	print "Rotated"
	PublishTwist(0, 0)

if __name__ == '__main__':

	global map_ready, occupancyGrid, stop
	global goal_ready, goal
	global x, y, theta, stop
	global twist_pub, wheel_rad, robot_rad, odom_list, bumper
	global map_origin, grid_res, grid_origin

	print "starting"

	rospy.init_node('Lab_4_node')

	map_ready = 0
	goal_ready = 0
	stop = 0
	goal = Point()
	occupancyGrid = None
	wheel_rad = .0381
	robot_rad = .2286
	bumper = 0

	rospy.Subscriber('map', OccupancyGrid, MapCallback)
	rospy.Subscriber('odom', Odometry, OdometryCallback) 
	#rospy.Subscriber('map_metadata', MapMetaData, MapMetaCallback)
	rospy.Subscriber('move_base_simple/goal', PoseStamped, GoalCallback)
	#rospy.Subscriber('initialpose', PoseWithCovarianceStamped, InitialPoseCallback)
	twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1) 
	exp_pub = rospy.Publisher('expanded_map', OccupancyGrid, queue_size=1)
	res_pub = rospy.Publisher('lower_res_map', OccupancyGrid, queue_size=1)

	odom_list = tf.TransformListener()
	rospy.sleep(2)
	#rospy.Timer(rospy.Duration(.01), timerCallback)

	print "Starting Lab 4"
	start = Point()
	start.z = 0

	while not map_ready:
		time.sleep(.3)
		print "please publish map"

	while 1:
		while not goal_ready:
			time.sleep(.3)
			print "waiting"

		goal_ready = 0
		expanded_map, lower_res_map = ObstacleExpan.ExpandMap(occupancyGrid)
		res_pub.publish(lower_res_map)
		exp_pub.publish(expanded_map)
		start.x = x
		start.y = y
		print "start", x, y
		print "goal", goal.x, goal.y
		stop = 0
		
		try:
			path = A_Star.GetPath(expanded_map, start, goal)
			waypoints = A_Star.Waypoints(path)
			for i in range (1, len(waypoints)):
				newx, newy = Waypoint_math.TranslateWaypoint(expanded_map, waypoints[i])
				turnAngle = Waypoint_math.ChooseTurnDirection(newx, newy, x, y, theta)
				print turnAngle
				Rotate(turnAngle)
				driveDistance = Waypoint_math.ChooseDriveDistance (newx, newy, x, y, theta)
				print driveDistance
				DriveStraight(.4, driveDistance)
				if (stop == 1):
					break
				#check for obstacles/change in map
		except (A_Star.NoPathError):
			print "No path error!"
			continue

	print "Lab 4 complete!"
