#!/usr/bin/env python3
#Deliverable 1 Task 3b
import rospy
from sensor_msgs.msg import LaserScan
import serial
import math

rospy.init_node('lidar')

#Connect to the lidar over serial
lidar = serial.Serial('/dev/ttyUSB0', baudrate= 230400)
#Using the LD19 dev manual to extract specific information
starter = 0x54
pub = rospy.Publisher('lidar', LaserScan, queue_size=100)

def talker():
    last_scan_timestamp = -1
    
    while not rospy.is_shutdown():
        scanr = LaserScan()
        scanr.ranges = []
        scanr.intensities = []
        if(int.from_bytes(lidar.read(1), byteorder='little') == starter):
            if(int.from_bytes(lidar.read(1), byteorder='little') == 0x2C):
                length = 0x0C
                speed = int.from_bytes(lidar.read(2), byteorder='little')
                start_ang = int.from_bytes(lidar.read(2), byteorder='little')

                for i in range(length):
                    data1 = lidar.read(2) #distance          
                    data2 = lidar.read() #Intensity
                    distance = int.from_bytes(data1, byteorder="little")
                    scanr.ranges.append(distance*.001)
                    scanr.intensities.append(int.from_bytes(data2, "little"))

                end_ang = int.from_bytes(lidar.read(2), byteorder='little')
                timestamp = int.from_bytes(lidar.read(2), byteorder='little')

                if last_scan_timestamp == -1:
                    last_scan_timestamp = timestamp

                crc_check = int.from_bytes(lidar.read(1), byteorder='little') 

                now = rospy.Time.now()
                scanr.header.stamp = now
                scanr.header.frame_id = 'laser.frame'
                scanr.angle_min = math.radians(start_ang * 0.01)
                scanr.angle_max = math.radians(end_ang * 0.01)
                scanr.angle_increment = (scanr.angle_max - scanr.angle_min) / (length - 1)
                scanr.time_increment = (scanr.angle_increment / math.radians(speed))
                scanr.scan_time = timestamp - last_scan_timestamp
                scanr.range_min = 0
                scanr.range_max = 100
                pub.publish(scanr)

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
         pass
