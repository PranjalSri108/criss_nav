#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import actionlib
import time
import math

current_aruco_id = None
previous_aruco_id = None
move_base_client = None
initial_orientation = None
current_orientation = None
current_position = [0, 0, 0]  # [x, y, yaw]
i = True

def aruco_callback(msg):
    global current_aruco_id, previous_aruco_id
    current_aruco_id = msg.data
    
    if current_aruco_id != previous_aruco_id and current_aruco_id == 15:
        if i == True:
            time.sleep(3)
        calculate_move_left()
    elif current_aruco_id != previous_aruco_id and current_aruco_id == 16:
        stop_robot()
    
    previous_aruco_id = current_aruco_id

def odom_callback(msg):
    global current_orientation, initial_orientation, current_position
    pose = msg.pose.pose
    current_orientation = pose.orientation
    current_position[0] = pose.position.x
    current_position[1] = pose.position.y
    # Convert quaternion to yaw
    euler = tf.transformations.euler_from_quaternion([
        current_orientation.x,
        current_orientation.y,
        current_orientation.z,
        current_orientation.w
    ])
    current_position[2] = euler[2]  # yaw

    if initial_orientation is None:
        initial_orientation = current_orientation

def calculate_move_left():
    i = False
    global move_base_client, initial_orientation, current_position
    if initial_orientation is not None:
        rospy.loginfo(f"New ArUco ID detected: {current_aruco_id}. Calculating new nav goal.")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        initial_yaw = current_position[2]
        left_x = -math.sin(initial_yaw) * 10.0
        left_y = math.cos(initial_yaw) * 10.0

        target_x = current_position[0] + left_x
        target_y = current_position[1] + left_y

        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, initial_yaw))
        
        move_base_client.send_goal(goal)
        rospy.loginfo(f"Current bot position: x={current_position[0]:.2f}, y={current_position[1]:.2f}")
        rospy.loginfo(f"Sent new nav goal: x={target_x:.2f}, y={target_y:.2f}")

def stop_robot():
    global is_moving_left, move_base_client
    rospy.loginfo("Stopping the robot")
    move_base_client.cancel_all_goals()
    is_moving_left = False

def main():
    global move_base_client
    rospy.init_node('robot_controller', anonymous=True)
    
    rospy.Subscriber('/aruco_detect/id', Int32, aruco_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
