#!/usr/bin/env python

import rospy, tf

from nav_msgs.msg import OccupancyGrid

import math
import time

def ExpandMap(occGrid):

	low_res_grid = occGrid(occGrid.header, occGrid.info, [])
	low_res_grid.info.resolution = .15
	low_res_grid.info.width = int(math.floor(float(occGrid.info.width)/2))
	low_res_grid.info.height = int(math.floor(float(occGrid.info.height)/2))

	width = low_res_grid.info.width
	height = low_res_grid.info.height

	#if width is odd
	if width % 2 == 1:
	#othervariable is width+1
		new_width = width + 1
	#else othervariable is width
	else:
		new_width = width

	low_res_grid.data = [-1]*width*height

	for i in range (0, height):
		for j in range (0, width):
			print i, j
			if occGrid.data[(j*2) + (width*2 * i*2)] >= 1 or \
				occGrid.data[(j*2) + (width*2 * ((i*2)+1))] >= 1 or \
				occGrid.data[(j*2)+1 + (width*2 * i*2)] >= 1 or \
				occGrid.data[(j*2)+1 + (width*2 * ((i*2)+1))] >= 1:
				low_res_grid.data[j + (new_width * i)] = 100 

			elif occGrid.data[(j*2) + (width*2 * i*2)] == 0 and \
				occGrid.data[(j*2) + (width*2 * ((i*2)+1))] == 0 and \
				occGrid.data[(j*2)+1 + (width*2 * i*2)] == 0 and \
				occGrid.data[(j*2)+1 + (width*2 * ((i*2)+1))] == 0:
				low_res_grid.data[j + (new_width * i)] = 0 
			else:
				low_res_grid.data[j + (new_width * i)] = -1 

			#time.sleep(2)

	expan_grid = occGrid(low_res_grid.header, low_res_grid.info, low_res_grid.data)
	expan_data = list(low_res_grid.data)

	print "expanding"
	for i in range (0, height):
		for j in range (0, width):
			if (low_res_grid.data[j + (width * i)] >= 1):
				for k in range (j - 2, j + 3):
					for l in range (i - 2, i + 3):
						if (k > 0 and k < width and l > 0 and l < height):
							expan_data[k + (width * l)] = 100

	expan_grid.data = tuple(expan_data)
	print "expanded"
	return expan_grid, low_res_grid

def MapCallback(occupancy):
	print "ready"
	global mapReady, occGrid
	mapReady = 1
	occGrid = occupancy

if __name__ == '__main__':

	global mapReady, occGrid
	rospy.init_node('Obstacle_Expan_Node')
	mapReady = 0
	occGrid = None
	rospy.Subscriber('map', occGrid, MapCallback)
	occ_pub = rospy.Publisher('expandedMap', occGrid, queue_size=1)
	res_pub = rospy.Publisher('resMap', occGrid, queue_size=1)
	odom_list = tf.TransformListener()
	rospy.sleep(rospy.Duration(1, 0))

	while not mapReady:
		time.sleep(.3)
		print mapReady
	expan_grid, low_res_grid = ExpandMap(occGrid)
	print "expanded"
	occ_pub.publish(expan_grid)
	res_pub.publish(low_res_grid)
	time.sleep(2)
