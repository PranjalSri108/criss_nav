#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32

bridge = CvBridge()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
aruco_params = cv2.aruco.DetectorParameters()
id_pub = None

def image_callback(data):
    global id_pub
    
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Detect ArUco markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, rejected = detector.detectMarkers(cv_image)

    # If markers are detected
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        
        for i, corner in enumerate(corners):
            center = np.mean(corner[0], axis=0)
            detected_id = ids[i][0]
            print(f"Detected ArUco marker ID {detected_id} at position {center}")
            
            # Publish the detected ID
            id_pub.publish(Int32(detected_id))

    cv2.imshow("ArUco Detection", cv_image)
    cv2.waitKey(1)

def main():
    global id_pub
    
    rospy.init_node('aruco_detector', anonymous=True)
    
    id_pub = rospy.Publisher('/aruco_detect/id', Int32, queue_size=10)
    image_sub = rospy.Subscriber("/kinect/rgb/image_raw", Image, image_callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
