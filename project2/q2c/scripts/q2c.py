import os
import rospy
import cv2
import numpy as np

currentdir = os.path.dirname(os.path.abspath(__file__))

CALIBRATION_PATH = f'{currentdir}/../../q2b/scripts/calibration.npy'
DISTORTION_PATH = f'{currentdir}/../../q2b/scripts/distortion.npy'


def get_marker_points(marker_size):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    return marker_points

def detect():
    frame = cv2.imread(f'{currentdir}/../data/sBXJulQ.jpeg')
    success = detect_from_image(frame, True)
    print(success)
    cv2.imwrite(f"{currentdir}/q2c_test_detection.png", frame)

def detect_from_image(frame, test=False):
    ardict = cv2.aruco.DICT_4X4_50
    if test:
        ardict = cv2.aruco.DICT_6X6_250

    dictionary = cv2.aruco.getPredefinedDictionary(ardict)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejectedImgPoints = detector.detectMarkers(frame)
    id_color = (0, 255, 0)  

    # Draw the detected markers with the specified ID color
    if ids is not None:
        for corner, id in zip(corners, ids):
            cv2.aruco.drawDetectedMarkers(frame, [corner], borderColor=id_color)
            cv2.putText(frame, str(id), tuple(corner[0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, id_color, 2)
    
    camera_matrix = []
    dist_coeffs = []

    if test:
        camera_matrix = np.array([[1000, 0, frame.shape[1]/2],
                                [0, 1000, frame.shape[0]/2],
                                [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
    else:
        camera_matrix = np.load(CALIBRATION_PATH)
        dist_coeffs = np.load(DISTORTION_PATH)
    
    detection = False
    if len(corners) > 0:
        for i in range(len(ids)):
            detection = True
            retval, rvec, tvec = cv2.solvePnP(get_marker_points(10), corners[i], camera_matrix, dist_coeffs)
            if retval:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 7)
    return detection

def main():
    #cv2.namedWindow("w1", cv2.WINDOW_AUTOSIZE)
    #videoCaptureObject = cv2.VideoCapture(1, apiPreference=cv2.CAP_V4L2)
    videoCaptureObject = cv2.VideoCapture(0)

    if videoCaptureObject.isOpened():
        try:
            while True:
                ret, image = videoCaptureObject.read()
                if ret:
                    success = detect_from_image(image)
                    print(success)
                    if success:
                        cv2.imwrite(f"{currentdir}/q2c_latest_detection.png", image)
                    else:
                        cv2.imwrite(f"{currentdir}/q2c_latest_frame.png", image)
                    #cv2.imshow("w1", image)
                    key = cv2.waitKey(100)
                    if key == 27: # exit on ESC
                        break
        except:
            pass
        #cv2.destroyWindow("w1")

if __name__=="__main__":

    main()
    #detect()
