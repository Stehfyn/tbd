#!/usr/bin/env python3
#Deliverable 2 q1b

import rospy
import sys
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
from ScanContext.ScanContext import ScanContext

g_ScanContext = None

def callback(msg): # interprets the info from the lidar and adds this data to the global context
    global g_ScanContext
    if g_ScanContext.still_scanning():
        g_ScanContext.add_laser_scan(msg)

def segment_data(): #Seperates the data into 2 clusters, inside the room and outside of the room
    sc = ScanContext()
    cluster = DBSCAN*(.5,200).fit(ScanContext)
    sc.plot_point_cloud()
    

def main():
    rospy.init_node('q1b', anonymous=True)
    rospy.Subscriber('lidar', LaserScan, callback)
    segment_data()


if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass