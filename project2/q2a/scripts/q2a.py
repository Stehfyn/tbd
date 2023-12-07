#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Joy, Image
import datetime

snaps = []
imageBuffer = None
videoCaptureObject = None
last_capture_time = None
capture_delay = 1 / 2

def open_cam() -> bool:
    global videoCaptureObject
    videoCaptureObject = cv2.VideoCapture(0, apiPreference=cv2.CAP_V4L2)
    return videoCaptureObject.isOpened()

def release_cam():
    global videoCaptureObject
    videoCaptureObject.release()

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
    global snaps
    global last_capture_time

    if data.buttons[0] and is_capture_delay_over():
        if open_cam():
            returnValue, capturedFrame = videoCaptureObject.read()
            if returnValue:
                snaps += [capturedFrame]
                last_capture_time = datetime.datetime.now()
            release_cam()

def snapshot_phase():
    global snaps
    while not rospy.is_shutdown() and len(snaps) < 4:
        pass

def show_snaps():
    global snaps

    for snap in snaps:
        cv2.imshow('snap', snap)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def write_snaps():
    global snaps

    for i, snap in enumerate(snaps):
        cv2.imwrite(f"snap_{str(i * 90)}.jpg", snap)

def main():
    try:
        rospy.init_node('q1a', anonymous=True)
        rospy.Subscriber("/joy", Joy, controller_listener)
        snapshot_phase()
        #show_snaps()
        write_snaps()

    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()