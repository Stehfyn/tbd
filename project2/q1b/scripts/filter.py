from ScanContext.ScanContext import ScanContext
import math
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler

# get lidar data
sc = ScanContext()
sc.deserialize("data/LidarScanContext.txt")

# Format data for dbSCAN
scan = []
for i in sc.data:
    row = []
    row.append(i[1] * math.cos(i[0]))
    row.append(i[1] * math.sin(i[0]))
    scan.append(row)

# send data to DB scan
scan = StandardScaler().fit_transform(scan)

# Adjust for clustering
db = DBSCAN(eps=0.04, min_samples=10).fit(scan)
labels = db.labels_

polar = []
for j, i in zip(labels, sc.data):
    if j == 0:
        row = []
        row.append(i[0])
        row.append(i[1])
        polar.append(row)

X, Y = [], []	
for i in polar:
	X.append(i[1] * math.cos(i[0]))
	Y.append(i[1] * math.sin(i[0]))
        
# Plot
plt.plot(X, Y,'ko')
plt.show()
