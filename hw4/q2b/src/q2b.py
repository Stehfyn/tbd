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
#!/usr/bin/env python3
import rospy
import roslib
from hw4_msgs.msg import CameraLocalization
from nav_msgs.msg import Odometry
from geometry_msgs import Twist

CONST_SPEED = 0.2

#Subscribing to the /pose topic to access the rotary encoders
def pose_gather(msg):
    print(msg.pose.pose) #Will print the pose of the robot 
    return msg.pose.pose

def print_sigma(msg):
    print(msg.CameraLocalization.distance_separation)

def main():
    twist = Twist()
    print('Start')
    rospy.init_node('pose_gather', anonymous=True)
    pose = rospy.Subscriber('/pose', Odometry, pose_gather)
    rospy.spin()
    pub = rospy.Publisher('pioneer/cmd_vel', Twist, queue_size= 10)
    twist.linear.x = 1*CONST_SPEED
    while(pose < pose+.3):
        pub.publish(twist)

    ## Print current sigma
    sigma = rospy.Subscriber('cam_local', CameraLocalization, print_sigma)
    

    


if __name__ == "__main__":
    main()