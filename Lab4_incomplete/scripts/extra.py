
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
	
