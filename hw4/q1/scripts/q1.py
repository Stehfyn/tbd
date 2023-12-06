import rospy
from hw4_msgs.msg import CameraLocalization
import cv2
import numpy as np
import os

CALIBRATION_PATH = 'calibration.npy'
DISTORTION_PATH = 'distortion.npy'

CALIBRATION_PATH = os.path.abspath(__file__ + f"/../{CALIBRATION_PATH}")
DISTORTION_PATH = os.path.abspath(__file__ + f"/../{DISTORTION_PATH}")
camera_matrix = np.load(CALIBRATION_PATH)
dist_coeffs = np.load(DISTORTION_PATH)
def get_marker_points(marker_size):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    return marker_points

def detect_from_image(frame):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejectedImgPoints = detector.detectMarkers(frame)
    id_color = (0, 255, 0)  

    # Draw the detected markers with the specified ID color
    if ids is not None:
        for corner, id in zip(corners, ids):
            cv2.aruco.drawDetectedMarkers(frame, [corner], borderColor=id_color)
            cv2.putText(frame, str(id), tuple(corner[0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, id_color, 2)
    
    '''
    camera_matrix = np.array([[1000, 0, frame.shape[1]/2],
                              [0, 1000, frame.shape[0]/2],
                              [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
    '''

    print(camera_matrix)
    #model_points = np.array([(0.0, 0.0, 0.0),(0.0, 1.0, 0.0), (1.0, 0.0, 0.0),(1.0, 1.0, 0.0)])
    
    cam_local = None

    if len(corners) > 0:
        for i in range(len(ids)):
            retval, rvec, tvec = cv2.solvePnP(get_marker_points(10), corners[i], camera_matrix, dist_coeffs)
            if retval:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 7)

                print('\n')
                print(rvec)
                print(tvec)
                print(tvec[2])
                print('\n')

                marker_center_x = np.mean(corners[i][0][:, 0])
        
                # Get the center of the image frame
                image_center_x = frame.shape[1] / 2

                # Calculate horizontal pixel distance
                pixel_distance = marker_center_x - image_center_x

                # Compute alignment error (α)
                alpha = (pixel_distance / frame.shape[1]) * 60  # frame.shape[1] is the width of the image in pixels
                print(alpha)
                cam_local = CameraLocalization()
                cam_local.alignment_error = alpha # in angles, negative is left in camera space
                cam_local.distance_separation = (tvec[2] / 2) * 0.01 # to meters
     
    if cam_local != None:
        return True, cam_local
    else:
        return False, []

def main():
    cv2.namedWindow("w1", cv2.WINDOW_AUTOSIZE)
    videoCaptureObject = cv2.VideoCapture(-1)
    pub = rospy.Publisher('cam_local', CameraLocalization, queue_size=1)
    rospy.init_node('q1', anonymous=True)
    if videoCaptureObject.isOpened():
        try:
            while True:
                ret, image = videoCaptureObject.read()
                if ret:
                    (success, cam_local) = detect_from_image(image)
                    if success:
                        pub.publish(cam_local)

                    cv2.imshow("w1", image)
                    key = cv2.waitKey(1)
                    if key == 27: # exit on ESC
                        break
        except:
            pass
        cv2.destroyWindow("w1")

if __name__=="__main__":
    main()