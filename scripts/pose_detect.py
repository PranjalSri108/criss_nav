#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ArUcoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/kinect/rgb/camera_info", CameraInfo, self.camera_info_callback)
        
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()
        
        try:
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_detector = None

        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.5  # Size of the ArUco marker in meters

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, data):
        if not hasattr(self, 'camera_matrix') or self.camera_matrix is None:
            rospy.logwarn("Camera info not received yet")
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        # Detect ArUco markers
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        if self.aruco_detector:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # If markers are detected
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            for i in range(len(ids)):
                marker_corners = corners[i][0]
                
                if marker_corners.shape[0] != 4:
                    rospy.logwarn(f"Marker {ids[i][0]} doesn't have exactly 4 corners. Skipping.")
                    continue

                # Define 3D points of the marker in its own coordinate system
                obj_points = np.array([
                    [-self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)

                marker_corners = marker_corners.reshape((4,2)).astype(np.float32)

                try:
                    _, rvec, tvec = cv2.solvePnP(obj_points, marker_corners, self.camera_matrix, self.dist_coeffs)
                except cv2.error as e:
                    rospy.logerr(f"Error in solvePnP for marker {ids[i][0]}: {str(e)}")
                    continue
                
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                
                # Convert rotation vector to euler angles
                rmat, _ = cv2.Rodrigues(rvec)
                euler_angles = cv2.RQDecomp3x3(rmat)[0]
                
                print(f"Detected ArUco marker ID {ids[i][0]}:")
                print(f"  Position: x={tvec[0][0]:.3f}, y={tvec[1][0]:.3f}, z={tvec[2][0]:.3f}")
                print(f"  Orientation: roll={np.degrees(euler_angles[0]):.2f}°, pitch={np.degrees(euler_angles[1]):.2f}°, yaw={np.degrees(euler_angles[2]):.2f}°")

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