#!/usr/bin/env python3

# GLOBAL CONSTANTS
# 0.1 seconds
#WAIT_TIME = 100
# 1.5 seconds
#WAIT_TIME = 1500
# 3 seconds
WAIT_TIME = 3000
# 5 seconds
#WAIT_TIME = 5000


# Imports
import rospy
import datetime
# Send images as a message
from sensor_msgs.msg import Image, Joy

# This package acts as a bridge between OpenCV and ROS
from cv_bridge import CvBridge
import cv2

# Create the name of our publisher node
publisherNodeName='camera_sensor_publisher'

# Name of the topic we are messaging over
topicName='camera_topic'

# Initialize the node
rospy.init_node(publisherNodeName, anonymous=True)

# Create a publisher object
# Specifies the name of the topic, the topic type, and buffer size
publisher=rospy.Publisher(topicName, Image, queue_size=1)

# Rate of the transmitted messages
rate = rospy.Rate(30)

# Create video capture object
videoCaptureObject=cv2.VideoCapture(0)
videoCaptureObject.set(cv2.CAP_PROP_FPS, 30)

# Create the bridge object
# This will convert OpenCV Images to ROS image messages
bridgeObject=CvBridge()

# The first picture is very dark, take a picture at init to remove
returnValue, capturedFrame = videoCaptureObject.read()

last_capture_time = None
capture_delay = 1 / 2

def is_capture_delay_over():
    global last_capture_time
    global capture_delay

    if last_capture_time != None:
        dur = datetime.datetime.now() - last_capture_time
        return dur.total_seconds() > capture_delay
    else:
        return True

def controller_listener(data):
    global videoCaptureObject
    global last_capture_time

    if data.buttons[0]:
        rospy.loginfo('Button pressed')
        if is_capture_delay_over():
            rospy.loginfo('capture delay over')

            returnValue, capturedFrame = videoCaptureObject.read()
            if returnValue:
                rospy.loginfo('Video frame captured and published')
                imageToTransmit=bridgeObject.cv2_to_imgmsg(capturedFrame)
                publisher.publish(imageToTransmit)
                last_capture_time = datetime.datetime.now()


rospy.loginfo("Press A to take picture")
rospy.Subscriber("/joy", Joy, controller_listener)
while not rospy.is_shutdown():

	try:
		pass
		
	except KeyboardInterrupt:
		break
		
cv2.destroyAllWindows()
