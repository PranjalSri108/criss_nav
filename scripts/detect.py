#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArUcoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_raw", Image, self.callback)  
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.aruco_params = cv2.aruco.DetectorParameters()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Detect ArUco markers
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, rejected = detector.detectMarkers(cv_image)

        # If markers are detected
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            for i, corner in enumerate(corners):
                center = np.mean(corner[0], axis=0)
                print(f"Detected ArUco marker ID {ids[i][0]} at position {center}")

        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('aruco_detector', anonymous=True)
    aruco_detector = ArUcoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
