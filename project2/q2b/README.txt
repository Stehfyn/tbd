|~|~|~|~|~|~|~|~|~| QUESTION 2b README |~|~|~|~|~|~|~|~|~|

This package contains two scripts:
 - camera_publisher.py
 - check_calibration.py

-----------------------------------------------------------
camera_publisher.py:
USAGE:
	rosrun q2b camera_publisher.py

INFO:
 - Takes a picture on keyboard input and transmits the 
   image over the ROS topic 'camera_topic'

-----------------------------------------------------------
check_calibration.py:
USAGE:
	rosrun q2b check_calibration.py

INFO:
- Recives images over ROS topic 'camera_topic'
- Determines if the image contains a checkerboard for
  calibration or not.
- Re-calibrates calibration matrix every 3 checkerboard 
  images send.
- Stores calibration details as local variables

NOTES:
- Requires matrix dimensions to be set on line 5.