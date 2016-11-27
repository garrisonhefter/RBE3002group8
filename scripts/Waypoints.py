import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

#import A_Star

def Waypoints(Still_Point_List):

	# found from online reference http://www.redblobgames.com/pathfinding/a-star/implementation.html#sec-1-4

	#waypointpub = rospy.Publisher('Waypoint_Cells', GridCells, queue_size=1)
	#WaypointCells = GridCells()
	#WaypointCells.cell_width = 1
	#WaypointCells.cell_height = 1
	#WaypointCells.cells = [Point()]
	#WaypointCells.header.frame_id = 'map'

	n = 0
	Still_Point_List = Still_Point_List
	print Still_Point_List

	for item in Still_Point_List:
		current_x = item.x
		current_y = item.y
		next_point = Still_Point_List[i+1]
		next_x = next_point.x
		next_y = next_point.y
		print "current x", current_x
		print "current y", current_y
		print "next x", next_x
		print "next Y", next_y


	#stack overflow version of placing values in the object

	#	if (next_x - current_x != 0):
	#		WaypointCells.cells[0].x = current_x
	#		WaypointCells.cells[0].y = current_y
	#		WaypointCells.cells[0].z = 0
	#		gridPub.publish(WaypointCells)
	#		n += 1
	#	elif (next_y - current_y != 0):
	#		WaypointCells.cells[0].x = current_x
	#		WaypointCells.cells[0].y = current_y
	#		WaypointCells.cells[0].z = 0
	#		gridPub.publish(WaypointCells)
	#		n += 1


if __name__ == '__main__':

	#Node initialization and Subscriber
	rospy.init_node('tstephen_Lab_3_Waypoint_node')
	rospy.Subscriber('path', GridCells, Waypoints, queue_size=1)

	trial = [(0,0,0), (1,1,1), (2,2,2,) (3,3,3) (2,3,2)]
	print "Initialize Waypoints"
	Waypoints(trial)
	print "Waypoints finished"
