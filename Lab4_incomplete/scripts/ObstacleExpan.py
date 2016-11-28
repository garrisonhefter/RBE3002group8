#!/usr/bin/env python

import rospy, tf

from nav_msgs.msg import OccupancyGrid

import math
import time

def ExpandMap(occupancyGrid):

	lowerResGrid = OccupancyGrid(occupancyGrid.header, occupancyGrid.info, [])

	lowerResGrid.info.resolution = .15
	lowerResGrid.info.width = int(math.floor(float(occupancyGrid.info.width)/2))
	lowerResGrid.info.height = int(math.floor(float(occupancyGrid.info.height)/2))

	width = lowerResGrid.info.width
	height = lowerResGrid.info.height

	#if width is odd
	#othervariable is width+1
	#else othervariable is width
	if width % 2 == 1:
		new_width = width + 1
	else:
		new_width = width

	lowerResGrid.data = [-1]*width*height

	for i in range (0, height):
		for j in range (0, width):
			print i, j
			if occupancyGrid.data[(j*2) + (width*2 * i*2)] >= 1 or \
				occupancyGrid.data[(j*2) + (width*2 * ((i*2)+1))] >= 1 or \
				occupancyGrid.data[(j*2)+1 + (width*2 * i*2)] >= 1 or \
				occupancyGrid.data[(j*2)+1 + (width*2 * ((i*2)+1))] >= 1:
				lowerResGrid.data[j + (new_width * i)] = 100 #put other variable here
			elif occupancyGrid.data[(j*2) + (width*2 * i*2)] == 0 and \
				occupancyGrid.data[(j*2) + (width*2 * ((i*2)+1))] == 0 and \
				occupancyGrid.data[(j*2)+1 + (width*2 * i*2)] == 0 and \
				occupancyGrid.data[(j*2)+1 + (width*2 * ((i*2)+1))] == 0:
				lowerResGrid.data[j + (new_width * i)] = 0 #put other variable here
			else:
				lowerResGrid.data[j + (new_width * i)] = -1 #put other variable here
			#time.sleep(2)

	expandedGrid = OccupancyGrid(lowerResGrid.header, lowerResGrid.info, lowerResGrid.data)

	expandedData = list(lowerResGrid.data)

	print "expanding"
	for i in range (0, height):
		for j in range (0, width):
			if (lowerResGrid.data[j + (width * i)] >= 1):
				for k in range (j - 2, j + 3):
					for l in range (i - 2, i + 3):
						if (k > 0 and k < width and l > 0 and l < height):
							expandedData[k + (width * l)] = 100

	expandedGrid.data = tuple(expandedData)
	print "expanded"
	return expandedGrid, lowerResGrid


def MapCallback(occupancy):
	print "ready"
	global mapReady, occupancyGrid
	
	mapReady = 1
	occupancyGrid = occupancy

if __name__ == '__main__':

	global mapReady, occupancyGrid

	rospy.init_node('doop')

	mapReady = 0
	occupancyGrid = None

	rospy.Subscriber('map', OccupancyGrid, MapCallback)
	occPub = rospy.Publisher('expandedMap', OccupancyGrid, queue_size=1)
	resPub = rospy.Publisher('resMap', OccupancyGrid, queue_size=1)

	odom_list = tf.TransformListener()

	rospy.sleep(rospy.Duration(1, 0))

	print "gonna wait"
	while not mapReady:
		time.sleep(.3)
		print mapReady

	print "doing the thing"
	expandedGrid, lowerResGrid = ExpandMap(occupancyGrid)
	print "expanded"
	occPub.publish(expandedGrid)
	resPub.publish(lowerResGrid)
	time.sleep(2)
