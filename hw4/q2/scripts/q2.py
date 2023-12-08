#!/usr/bin/env python

import rospy 
import roslib
from geometry_msgs.msg import Twist
import sys
from hw4_msgs.msg import CameraLocalization

# Create publisher to transmit
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


def callback(position):
	global pub
	
	# Get ALPHA from topic
	# Negative alpha = right
	# Positive alpha = left
	alpha = position.alignment_error
	#print(alpha)
	#rospy.loginfo(alpha)
	
	
	if alpha < -3 or alpha > 3:
		turn(alpha)
	
	# Get a twist object to publish steering data
	#twist = Twist()
	#twist.angular.z = alpha
	#pub.publish(twist)
	
	
####### END CALLBACK() #########

def turn(angle):
	global pub
	twist = Twist()
	if angle < 0:
		twist.angular.z = 0.1
		print("Turning Left")
	if angle > 0:
		twist.angular.z = -0.1
		print("Turning Right")
	pub.publish(twist)

# Setup ROS node
def main():
	# Initalize node
	rospy.init_node('q1b', anonymous=True)
	
	# Subsribe to cam_local topic
	rospy.Subscriber('cam_local', CameraLocalization, callback)
	
	# Create publisher to transmit
	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	
if __name__ == '__main__':
    main()