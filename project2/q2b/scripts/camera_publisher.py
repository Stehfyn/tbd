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

# Send images as a message
from sensor_msgs.msg import Image

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
publisher=rospy.Publisher(topicName, Image, queue_size=60)

# Rate of the transmitted messages
rate = rospy.Rate(30)

# Create video capture object
videoCaptureObject=cv2.VideoCapture(0)

# Create the bridge object
# This will convert OpenCV Images to ROS image messages
bridgeObject=CvBridge()

# The first picture is very dark, take a picture at init to remove
returnValue, capturedFrame = videoCaptureObject.read()

# Loop for when node is running
while not rospy.is_shutdown():

	# Prompt user for feedback
	prompt = input("Show picture (y/n)? ")
	
	# Get image from camera
	returnValue, capturedFrame = videoCaptureObject.read()
	
	# If image was retrieved successfully
	if returnValue == True:
		# Publish image on ROS topic
		rospy.loginfo('Video frame captured and published')
		imageToTransmit=bridgeObject.cv2_to_imgmsg(capturedFrame)
		publisher.publish(imageToTransmit)
		
		# Display image if prompted
		if prompt != 'n':
			resized = cv2.resize(capturedFrame, (0, 0), fx = 0.5, fy = 0.5)
			cv2.imshow("Robot Vision", resized)
			cv2.waitKey(WAIT_TIME)
			cv2.destroyAllWindows()
		
	rate.sleep()











