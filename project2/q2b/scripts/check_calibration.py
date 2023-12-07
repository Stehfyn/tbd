#!/usr/bin/env python3

########## GLOBAL CONSTANTS ##########
# Define the dimensions of checkerboard 
CHECKERBOARD = (9, 6)

# Define the number of calibration images before performing calibration
IMAGE_THRESHOLD = 20

# Array for checkerboard images
checkerboardImages = []

# Arrays to store the calibration values
calibrationMatrix = []
distortionMatirx = []
rotationMatrix = []
translationMatrix = []

import os
currentdir = os.path.dirname(os.path.abspath(__file__))
# PATH FOR FILE SAVES
CALIBRATION_PATH = f'{currentdir}/calibration.npy'
DISTORTION_PATH = f'{currentdir}/distortion.npy'
IMAGE_PATH = f'{currentdir}/calibration.png'

# IMPORTS
import rospy
import numpy as np
import copy

# For sending messages in the form of images
from sensor_msgs.msg import Image

# Acts as a bridge to send cv2 images over ROS messages
from cv_bridge import CvBridge
import cv2

# Function that is called everytime a new message arrives
def callbackFunction(message):
	global checkerboardImages

	# Feedback for image recieved
	rospy.loginfo("image recieved")
	
	# Bridge object to convert ROS message to CV2
	bridgeObject = CvBridge()
	convertedImage = bridgeObject.imgmsg_to_cv2(message)
	#convertedImage = cv2.resize(convertedImage, (0, 0), fx = 0.5, fy = 0.5)
	
	# Grayscale image
	grayColor = cv2.cvtColor(convertedImage, cv2.COLOR_BGR2GRAY)
	
	# Detirmine if there is a checkerboard in the frame
	ret, corners = cv2.findChessboardCorners(grayColor, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
	
	# Enter if checkerboard is found
	if ret == True:
		# Prints positive to console and stores image in checkerboard images array
		rospy.loginfo("checkerboard!")
		checkerboardImages.append(convertedImage)
		
		# Every IMAGE_THRESHOLD of pictures taken will rerun calibration
		# OLD IMAGES ARE NEVER ERASED
		if len(checkerboardImages) % IMAGE_THRESHOLD == 0:
			rospy.loginfo("Starting calibration")
			createMatrix()
			checkerboardImages = []
			
			# Print calibration Matrix and distortion Matrix to console
			rospy.loginfo("Calibration Matrix:")
			for i in calibrationMatrix:
				rospy.loginfo(i)
			rospy.loginfo("Distortion Matrix:")
			for i in distortionMatirx:
				rospy.loginfo(i)
			
	else:
		rospy.loginfo("not checkerboard")
##### END CALLBACK FUNCTION #####
	
def createMatrix():
	#global checkImages
	global calibrationMatrix
	global distortionMatirx 
	global rotationMatrix
	global translationMatrix
	
	# Critera for calibration
	# Stop the iteration when specified  accuracy, epsilon, is reached or specified number of iterations are completed. 
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 

	# Vector for 3D points 
	threeDpoints = [] 

	# Vector for 2D points 
	twoDpoints = [] 

	# 3D points real world coordinates 
	objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32) 
	objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) 
	prev_img_shape = None
	
	# Loop through images
	for j in checkerboardImages:
		image = copy.copy(j)
	
		# Grayscale image
		grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		
		# Get corners of chessboard
		ret, corners = cv2.findChessboardCorners(grayImage, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
		
		# Get 3d points on chessboard
		threeDpoints.append(objectp3d) 

		# Refining pixel coordinates 
		# for given 2d points. 
		corners2 = cv2.cornerSubPix(grayImage, corners, (11, 11), (-1, -1), criteria) 
		
		# Append corners to 2d points
		twoDpoints.append(corners2)
		
		# Draw and display the corners 
		image = cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)
		

		#cv2.imshow('img', image)
		#cv2.imwrite('calibration_data/calibration.png', image)
		cv2.imwrite(IMAGE_PATH, image)
		#cv2.waitKey(0) 

	#scv2.destroyAllWindows() 

	h, w = image.shape[:2]
	
	# Perform and return calibration matricies
	ret, calibrationMatrix, distortionMatirx, rotationMatrix, translationMatrix = cv2.calibrateCamera(threeDpoints, twoDpoints, grayImage.shape[::-1], None, None)
	
	
	# Write calibartion and distortion matrix to files
	#with open('calibration_data/calibration.npy', 'wb') as f:
	with open(CALIBRATION_PATH, 'wb') as f:
		np.save(f, np.array(calibrationMatrix))
		
	#with open('calibration_data/distortion.npy', 'wb') as f:
	with open(DISTORTION_PATH, 'wb') as f:
		np.save(f, np.array(distortionMatirx))

##### END CREATE MATRIX FUNCTION #####

	
########## NODE INIT ##########
# Create the name of the subsriber node
subscriberNodeName='checkerboard_calibration'

# Topic name, change to the topic that the publisher is using
topicName='camera_topic'

# Anonymous appends a random number to the node name
rospy.init_node(subscriberNodeName, anonymous=True)

# Subscribe to the topic
# Arguments = (Topic name, message type, and function to be called when image is recieved)
rospy.Subscriber(topicName, Image, callbackFunction)

'''
while True:
	try:
		pass
	except:
		break
'''		
# Spin keeps node active after init
rospy.spin()

# Destroy any open windows on exit
#cv2.destroyAllWindows()
