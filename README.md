# CRISS NAV
## 2D LiDAR Mapping
We are going to use Hector_Slam to create a Map of our Gazebo environment with LiDAR
- Launching our bot in Gazebo environment:

```bash
cd /path/to/catkin_src_folder
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
We will now map a 3D Octomap of our environment using Hector_Slam and Kinect Camera Plugin
- But before launching the bot in gazebo, make sure the 'min' and 'max' params for Kinect plugin in gazebo_plugins.urdf.xacro are updated as given below

```xacro
   <!-- Plugin for Kinect sensor -->
   <gazebo reference="kinect">
       <sensor type="depth" name="camera1">
       <always_on>1</always_on>
       <visualize>true</visualize>             
       <camera>
           <horizontal_fov>1.047</horizontal_fov>  
           <image>
               <width>640</width>
               <height>480</height>
               <format>R8G8B8</format>
           </image>
           <depth_camera>

           </depth_camera>
           <clip>
                <near>0.1</near>        # These Params
                <far>100.0</far>
           </clip>
       </camera>
```
- Launch bot in Gazebo

``` bash
roslaunch criss_nav navros_aruco_control.launch
```

- Launch SLAM

```bash
roslaunch hector_slam_launch tutorial.launch
```

- Launch Octomap

```bash
roslaunch criss_nav octomap.launch
```

- Visualize in RViz

```bash
cd /criss_nav/rviz
rviz -d octomap.rviz
```
You can now map the environment using the Teleop Keyboard Control with the same settings

---
## Move_Base Navigation
We will now implement move_base on our bot to navigate though our 2D Gazebo world map.

- Tech Stack of move_base
  - Global Costmap Path Planner: Dijkstra algorithmn
  - Local Costmap Path Planner: DWA planner
