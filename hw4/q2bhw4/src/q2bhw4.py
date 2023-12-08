#!/usr/bin/env python3
import rospy
import roslib
import math
import time
from hw4_msgs.msg import CameraLocalization
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

'''
Q2B Objective
Now that the robot is aligned in the direction of the ArUco marker, move it 30 centimeters towards
ArUco marker. This can be achieved by utilizing the rotary encoders on the pioneer to measure how far
the wheels have moved over time. You must use these encoders as feedback to determine how far the robot
has traveled. To do this, subscribe to the /pose topic after running the p2os driver.launch file to obtain the
odometry messages which contain the rotary encoder information. See Odometry message type. Publish a
constant linear velocity of 0.20m/s to the /cmd vel topic to move the robot until it has traveled your desired
distance. Finally, print the distance the robot has traveled and the current value of δ.
'''

CONST_SPEED = 0.2
CONST_TIME = 0.13

# Create publisher
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Get initial position values
INIT_X, INIT_Y = 0.0, 0.0
initialize_bool = True

#Subscribing to the /pose topic to access the rotary encoders
def pose_gather(msg):
    global INIT_X
    global INIT_Y
    global initialize_bool
   
    if initialize_bool:
        INIT_X = msg.pose.pose.position.x
        INIT_Y = msg.pose.pose.position.y
        initialize_bool = False

    x = msg.pose.pose.position.x - INIT_X
    y = msg.pose.pose.position.y - INIT_Y

    dist = math.sqrt((x ** 2) + (y ** 2))
   
    #print("X:", x)
    #print("Y:", y)
    #print(dist, '\n')
    if dist < .03:
        go_forward(dist)
   
    '''
    xy = []
    #print(msg.pose.pose) #Will print the pose of the robot
    xy.append(msg.pose.pose.position.x)
    #xy.append(msg.pose[0])
    print(xy)
    return xy
    '''
   
def go_forward(dist):
    #print(dist)
    twist = Twist()
    twist.linear.x = CONST_SPEED
    pub.publish(twist)
   
    t1 = time.time()
    t2 = t1 + CONST_TIME
    while (t1 < t2):
        t1 = time.time()
        pub.publish(twist)


def print_sigma(msg):
    print(msg.CameraLocalization.distance_separation)
   
def callback(msg):
    print(msg.distance_separation)

def main():

    twist = Twist()
    print('Start')
    rospy.init_node('pose_gather', anonymous=True)
    #pose = rospy.Subscriber('/pose', Odometry, pose_gather)
    rospy.Subscriber('/pose', Odometry, pose_gather)
   
    # Subsribe to cam_local topic
    rospy.Subscriber('cam_local', CameraLocalization, callback)
   
   
    rospy.spin()
'''    
    if pose < .3:
    twist.linear.x = 1*CONST_SPEED
    pub.publish(twist)
    print("I am moving!")
'''
   
'''
    twist.linear.x = 1*CONST_SPEED
    pub = rospy.Publisher('pioneer/cmd_vel', Twist, queue_size= 1)
    while(pose < pose+.3):
        pub.publish(twist)
    rospy.spin()

    ## Print current sigma
    sigma = rospy.Subscriber('cam_local', CameraLocalization, print_sigma)
'''
   


if __name__ == "__main__":
    main()