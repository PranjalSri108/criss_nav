## CRISS NAV
### 2D LiDAR Mapping
We are going to use Hector_Slam to create a Map of our Gazebo environment with LiDAR
- Launching our bot in Gazebo environment:

```bash
$ cd /path/to/catkin_src_folder
```
``` bash
roslaunch criss_nav navros_aruco_control.launch
```

- Launch SLAM

```bash
roslaunch hector_slam_launch tutorial.launch
```

The RViz window will launch and you can start mapping by controlling the bot using Teleop Keyboard Control
- Launch Teleop Keyboard Control
  - Set Linear speed = 0.20
  - Set Angular speed = 0.25

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
---
## 3D Octomap 

