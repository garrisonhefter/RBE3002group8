#!/usr/bin/env python

import rospy, tf
import time
import Queue
from nav_msgs.msg import GridCells
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


#error exception
class NoPathError(Exception):
	# Found from Referenced source
	def __init__(self, number):
		self.number = number
	def __str__(self):
		return repr(self.number)
    

def GetData (x, y, gridCellsMap):
	#Path locations
	width = gridCellsMap.info.width
	height = gridCellsMap.info.height
	# checks parameter constraints
	if (x < 0 or x > width or y < 0 or y > height):
		return 1
	data_loc = (width * y) + x
	print width
	print height
	print x
	print y
	print data_loc
	return gridCellsMap.data[data_loc]

def GetNeighbors (a, gridCellsMap):
	#found from referenced source
	neighbors = []
	#Positive y = North
	if (GetData(a.x, a.y + 1, gridCellsMap) == 0):
		neighbors.append(Point(a.x, a.y + 1, 0))
	#Positive X = East
	if (GetData(a.x + 1, a.y, gridCellsMap) == 0):
		neighbors.append(Point(a.x + 1, a.y, 0))
	#Negative Y = South
	if (GetData(a.x, a.y - 1, gridCellsMap) == 0):
		neighbors.append(Point(a.x, a.y - 1, 0))
	#Negative X = West
	if (GetData(a.x - 1, a.y, gridCellsMap) == 0):
		neighbors.append(Point(a.x - 1, a.y, 0))
	return neighbors


def GetPath (gridMap, start_pos, goal_pos):
	#Publisher ('path')
	path_Publisher = rospy.Publisher('path', GridCells, queue_size=1) 

	trans_start, trans_goal = translatePoints(gridMap, start_pos, goal_pos)
	#Set up Queue and nodes for the path based off of the start and goal pos
	parents, costs, currNode = goalSearch(gridMap, start_pos, goal_pos)
	path = Path()
	poseStampedList = []
	i = 0
	pathList = []

	print "getting path"
	#inititate the path collection while the 
	while not (currNode.x == trans_start.x and currNode.y == trans_start.y): 
		print currNode
		pathList.append(currNode)
		currNode = parents[currNode]
		i += 1

	pathList.append(trans_start)
	path = GCfromList(gridMap,pathList)
	print "found path"
	path_Publisher.publish(path)
	return pathList

def GCfromList (gridMap,List):
	# from referenced Source and previous labs
	gridCells = GridCells()
	gridCells.cell_width = .1
	gridCells.cell_height = .1
	newList = []
	for i in range (0, len(List)):
		x = ((float(List[i].x)/ 10) + gridMap.info.origin.position.x)
		y = ((float(List[i].y)/ 10) + gridMap.info.origin.position.y)
		newList.append(Point(x, y, 0))
		print List[i].x
		print List[i].y
		print x
		print y
		#time.sleep(1)
	gridCells.cells = newList
	gridCells.header.frame_id = 'map'
	return gridCells
	
def HashPoint (a):
	return ("x"+str(a.x)+"y"+str(a.y)+"z"+str(a.z))

def Waypoints(List):

	#refer to Get Path setup
	print "waypoints!"
	#waypoint_pub = rospy.Publisher('waypoints', GridCells, queue_size=1)
	WaypointCells = []
	i = 0
	currState = 2
	#WaypointCells.append(List[0])

	for item in List:
		curr_pos = item
		#checks the queue until the end of the list
		if (i < len(List)-1):
			next_pos = List[i+1]
			i += 1

			#When x changes and y is constant
			if ((not(next_pos.x - curr_pos.x == 0)) and (next_pos.y - curr_pos.y == 0)):
				if (currState == 1):
					WaypointCells.append(curr_pos)
					#waypoints = GCfromList(WaypointCells)
					#waypoint_pub.publish(waypoints)
					#time.sleep(.2)
				currState = 0	
				
			#When y changes and x is constant
			elif ((next_pos.x - curr_pos.x == 0) and (not(next_pos.y - curr_pos.y == 0))):
				if (currState == 0):
					WaypointCells.append(curr_pos)
					#waypoints = GCfromList(WaypointCells)
					#waypoint_pub.publish(waypoints)
					#time.sleep(.2)
				currState = 1


			#Neither change 	
			else:
				print "something is broken"
				print curr_pos
				print next_pos

	WaypointCells.append(List[len(List)-1])
	#waypoints = GCfromList(WaypointCells)
	#waypoint_pub.publish(waypoints)
	#time.sleep(.5)
	print WaypointCells
	return WaypointCells


def translatePoints(gridMap, start_pos, goal_pos):

	trans_start = start_pos
	trans_goal = goal_pos

	trans_start.x = int(round((trans_start.x - gridMap.info.origin.position.x) * 10))
	trans_start.y = int(round((trans_start.y - gridMap.info.origin.position.y) * 10))

	trans_goal.x = int(round((trans_goal.x - gridMap.info.origin.position.x) * 10))
	trans_goal.y = int(round((trans_goal.y - gridMap.info.origin.position.y) * 10))

	return trans_start, trans_goal
	

def goalSearch (gridCellsMap, start_pos, goal_pos):

	print start, goal
	
	#Data collection found from reference source with minor adjustments
	parents = {}
	parents[start_pos] = None
	costs = {}
	costs[HashPoint(start_pos)] = 0
	frontierList = [start_pos]
	visitedList = []
	found = [start_pos]

	#The Frontier
	frontier = Queue.PriorityQueue()
	frontier.put((0, start_pos))

	#publishers
	pub_frontier = rospy.Publisher('frontier', GridCells, queue_size=1) 
	pub_visited = rospy.Publisher('visited', GridCells, queue_size=1) 

	print "Calculating Path"
	# ha no success, like this lab
	success = 0

	while not frontier.empty():
		#time.sleep(.03)
		#Use frontier and update lists to add to the 
		p, currNode = frontier.get()
		if currNode not in visitedList:
			print currNode
			frontierList.remove(currNode)
			visitedList.append(currNode)
			#publishing frontiers and visited
			frontiers = GCfromList(gridCellsMap,frontierList)
			pub_frontier.publish(frontiers)
			visited = GCfromList(gridCellsMap,visitedList)
			pub_visited.publish(visited)
			#if goal pos and currNode arent equal count it as sucess
			if (currNode.x == goal_pos.x and currNode.y == goal_pos.y): 
				success = 1
				break

			for neighbor in GetNeighbors(currNode, gridCellsMap):
				neighborCost = costs[HashPoint(currNode)] + 1
				#sets up priority and Heuristic evaluation
				if neighbor not in found or costs[HashPoint(neighbor)] > neighborCost:
					costs[HashPoint(neighbor)] = neighborCost
					priority = neighborCost + ((abs(neighbor.x - goal_pos.x) + abs(neighbor.y - goal_pos.y))*2) #CalcHeuristic here
					frontier.put((priority, neighbor))
					#When neighbor is not found list make sure to add it to the lists
					if neighbor not in found:
						frontierList.append(neighbor)
						found.append(neighbor)
					parents[neighbor] = currNode
	# if the process works return the values otherwise error message with exception
	if success:
		return parents, costs, currNode
	else:
		raise NoPathError("There is no path!")
