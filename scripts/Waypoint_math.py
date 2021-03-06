#!/usr/bin/env python


import rospy
import math
import time

def TurnDirection (goal_wp, x, y, theta):

	goal_x = goal_wp.x - x
	goal_y = goal_wp.y - y
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
	print goal_wp.x
	print goal_wp.y
	print goal_x
	print goal_y
	print theta
	print goal_theta
	time.sleep(10)

	return goal_theta

def DriveDistance (goal_wp, x, y, theta):
	return ((((x - goal_wp.x) ** 2) + ((y - goal_wp.y) ** 2)) ** (0.5))

def TranslateWaypoint(gridMap, point):

	trans_point = point
	print gridMap.info.origin.position.x
	print gridMap.info.origin.position.y
	print trans_point.x
	print trans_point.y
	trans_point.x = ((float(trans_point.x)/ 10) + gridMap.info.origin.position.x)
	trans_point.y = ((float(trans_point.y) / 10) + gridMap.info.origin.position.y)
	return trans_point
