from ScanContext.ScanContext import ScanContext
import math
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
import Utilities


# Lists
points = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
lines = [[0, 0], [0, 0], [0, 0], [0, 0]]

# Dictionaries
pointsDict= {
    "A": points[0], 
    "B": points[1],
    "C": points[2],
    "D": points[3],
    "E": points[4],
    "F": points[5]
}
linesDict= {
    "Lower Wall": lines[0], 
    "Right Wall": lines[1],
    "Upper Wall": lines[2],
    "Left Wall": lines[3]
}

def getCorners(filename):
	global points
	global lines
	global pointsDict
	global linesDict
	
	sc = ScanContext()
	sc.deserialize(filename)

	# Format data for dbSCAN
	scan = []
	for i in sc.data:
	    row = []
	    row.append(i[1] * math.cos(i[0]))
	    row.append(i[1] * math.sin(i[0]))
	    scan.append(row)

	# send data to DB scan
	scan = StandardScaler().fit_transform(scan)

	# Filter out noise
	db = DBSCAN(eps=0.04, min_samples=10).fit(scan)
	labels = db.labels_
	polar = []
	for j, i in zip(labels, sc.data):
		if j == 0 and i[0] < 6.282:
			row = []
			row.append(i[0])
			row.append(i[1])
			polar.append(row)


	# Re order polar coordinates 
	polar = sorted(polar, key=lambda x: x[0])
	i = 0
	index = 0
	distance = abs(polar[0][1] - polar[len(polar) -1][1])
	while i < (len(polar) - 1):
		temp = abs(polar[i][1] - polar[i + 1][1])
		if temp > distance:
			distance = temp
			index = i
		i = i + 1	
	i = index
	od = []
	while i < len(polar):
		od.append(polar[i])
		i = i + 1
	i = 0
	while i < index:
		od.append(polar[i])
		i = i + 1
		

	# Create cartiesia coordinates
	X, Y, orderCoords = [], [], []	
	for i in od:
		row = []
		a, b = 0, 0
		row.append(i[1] * math.cos(i[0]))
		row.append(i[1] * math.sin(i[0]))
		orderCoords.append(row)
		X.append(row[0])
		Y.append(row[1])
		
	# Set points E and F = last and first points in orderCoords
	pointsDict["F"] = orderCoords[0]
	pointsDict["E"] = orderCoords[len(orderCoords) - 1]

	grouping = 100
	segments = grouping
	i, j, acc = 0, 0, 0
	clusters = []
	while i < len(od):
		if i + grouping > len(od):
			j = grouping
			i = len(od)
		while j < grouping:
			acc = acc + od[i][1]
			i = i + 1
			j = j + 1
		clusters.append(acc / grouping)
		acc = 0
		j = 0

	# Get points of inflection
	i = 0
	minF = True
	maxF = False
	discard = []
	while i < len(clusters) - 1:
		# Find min
		if minF:
			if clusters[i] - clusters[i + 1] < 0:
				minF = False
				maxF = True
		if maxF:
			if clusters[i] - clusters[i + 1] > 0:
				 minF = True
				 maxF = False
				 discard.append(i)
		i = i + 1

	# Get final lines 
	linesDict["Lower Wall"] = Utilities.order_wls(Utilities.segemnt(orderCoords, 0, (discard[0] * segments) - 1))
	linesDict["Right Wall"] = Utilities.order_wls(Utilities.segemnt(orderCoords, (discard[0] + 1) * segments, (discard[1] * segments) - 1))
	linesDict["Upper Wall"] = Utilities.order_wls(Utilities.segemnt(orderCoords, (discard[1] + 1) * segments, (discard[2] * segments) - 1))
	linesDict["Left Wall"] = Utilities.order_wls(Utilities.segemnt(orderCoords, (discard[2] + 1) * segments, len(orderCoords)))

	# Find points of intersection
	pointsDict["A"] = Utilities.intersection(linesDict["Upper Wall"], linesDict["Left Wall"])
	pointsDict["B"] = Utilities.intersection(linesDict["Upper Wall"], linesDict["Right Wall"])
	pointsDict["C"] = Utilities.intersection(linesDict["Lower Wall"], linesDict["Right Wall"])
	pointsDict["D"] = Utilities.intersection(linesDict["Lower Wall"], linesDict["Left Wall"])
	
	ret = [pointsDict["A"], pointsDict["B"], pointsDict["C"], pointsDict["D"], pointsDict["E"], pointsDict["F"]]
	return ret

### END FUNCTION DEF ###
