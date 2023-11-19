#!/usr/bin/env python
#Deliverable 2 task 1
#Goal intrepret the LaserScan Data being fed into the '\lidar topic' store this information into a dataframe for future reference in the room find algorithm
#May need to publish the data frames to a new topic. 
import rospy
from sensor_msgs.msg import LaserScan
from ScanContext.ScanContext import ScanContext

g_ScanContext = None


def callback(msg): #will interpret laser scan objects into its respective parts
    global g_ScanContext
    if g_ScanContext.still_scanning():
        g_ScanContext.add_laser_scan(msg)


def process_lidar(): # Will listen to the 'lidar topic' and the callback function will be performed on each msg recieved
    global g_ScanContext
    g_ScanContext = ScanContext()

    rospy.init_node('processer',anonymous=True)
    datapoints = rospy.Subscriber("lidar", LaserScan, callback)

    while g_ScanContext.still_scanning():
        pass

    g_ScanContext.serialize()
    
def main():
    process_lidar()


if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass