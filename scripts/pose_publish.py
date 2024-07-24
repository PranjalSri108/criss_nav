#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

bridge = CvBridge()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
aruco_params = cv2.aruco.DetectorParameters()
id_pub = None
pose_pub = None
camera_matrix = None
dist_coeffs = None
marker_size = 1.0  # Size of the ArUco marker in meters

def camera_info_callback(msg):
    global camera_matrix, dist_coeffs
    camera_matrix = np.array(msg.K).reshape(3, 3)
    dist_coeffs = np.array(msg.D)

def image_callback(data):
    global id_pub, pose_pub, camera_matrix, dist_coeffs
    
    if camera_matrix is None:
        rospy.logwarn("Camera info not received yet")
        return

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
            marker_corners = corner[0]
            detected_id = ids[i][0]
            
            # Publish the detected ID
            id_pub.publish(Int32(detected_id))

            # Estimate pose
            obj_points = np.array([
                [-marker_size/2, marker_size/2, 0],
                [marker_size/2, marker_size/2, 0],
                [marker_size/2, -marker_size/2, 0],
                [-marker_size/2, -marker_size/2, 0]
            ], dtype=np.float32)

            _, rvec, tvec = cv2.solvePnP(obj_points, marker_corners, camera_matrix, dist_coeffs)

            # Draw axis for the ArUco marker
            cv2.drawFrameAxes(cv_image, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)

            # Convert rotation vector to euler angles
            rmat, _ = cv2.Rodrigues(rvec)
            euler_angles = cv2.RQDecomp3x3(rmat)[0]

            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_frame"  # Adjust this to your camera's frame
            pose_msg.pose.position.x = tvec[0][0]
            pose_msg.pose.position.y = tvec[1][0]
            pose_msg.pose.position.z = tvec[2][0]
            
            quat = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            pose_pub.publish(pose_msg)

            print(f"Detected ArUco marker ID {detected_id}:")
            print(f"  Position: x={tvec[0][0]:.3f}, y={tvec[1][0]:.3f}, z={tvec[2][0]:.3f}")
            print(f"  Orientation: roll={np.degrees(euler_angles[0]):.2f}°, pitch={np.degrees(euler_angles[1]):.2f}°, yaw={np.degrees(euler_angles[2]):.2f}°")

    cv2.imshow("ArUco Detection with Axes", cv_image)
    cv2.waitKey(1)

def main():
    global id_pub, pose_pub
    
    rospy.init_node('aruco_detector', anonymous=True)
    
    id_pub = rospy.Publisher('/aruco_detect/id', Int32, queue_size=10)
    pose_pub = rospy.Publisher('/aruco_detect/pose', PoseStamped, queue_size=10)
    
    image_sub = rospy.Subscriber("/kinect/rgb/image_raw", Image, image_callback)
    camera_info_sub = rospy.Subscriber("/kinect/rgb/camera_info", CameraInfo, camera_info_callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()