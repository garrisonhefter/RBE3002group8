#!/usr/bin/env python

# the thing josh said I needed
import rospy
import math
import time

def ChooseTurnDirection (goal_waypoint, x, y, theta):

	#goal_waypoint = waypointList[1]
	goal_x = goal_waypoint.x - x
	goal_y = goal_waypoint.y - y

	goal_theta = 0

	if (goal_x == 0):
		if (goal_y > 0):
			goal_theta = math.pi/2
		else:
			goal_theta = -math.pi/2
	else:
		goal_theta = theta - math.atan(goal_y/goal_x) 

	print "printing choose turn direction"
	print x
	print y
	print goal_waypoint.x
	print goal_waypoint.y
	print goal_x
	print goal_y
	print theta
	print goal_theta
	time.sleep(10)

	return goal_theta

def ChooseDriveDistance (goal_waypoint, x, y, theta):
	return ((((x - goal_waypoint.x) ** 2) + ((y - goal_waypoint.y) ** 2)) ** (0.5))

def TranslateWaypoint(gridMap, point):

	translatedPoint = point

	print gridMap.info.origin.position.x
	print gridMap.info.origin.position.y
	print translatedPoint.x
	print translatedPoint.y

	translatedPoint.x = ((float(translatedPoint.x)/ 10) + gridMap.info.origin.position.x)
	translatedPoint.y = ((float(translatedPoint.y) / 10) + gridMap.info.origin.position.y)

	return translatedPoint

# #Odometry Callback function
# def OdometryCallback(msg):
# 	#Current x, y, and theta
# 	global x, y, theta
# 	xPos = msg.pose.pose.position.x
# 	yPos = msg.pose.pose.position.y
# 	orientation = msg.pose.pose.orientation
# 	quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
# 	roll, pitch, yaw = euler_from_quaternion(quaternion)

# 	x = xPos
# 	y = yPos
# 	theta = yaw

# if __name__ == '__main__':

# 	global x, y, theta

# 	x, y, theta = 0

# 	rospy.Subscriber('odom', Odometry, OdometryCallback) 
