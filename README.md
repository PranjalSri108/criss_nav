# CRISS NAV
### Features
- 2D LiDAR Mapping
- Move_base Navigation
- 3D Octomap Mapping
- ArUco Marker Detection in stimulated environment
- ArUco based Navigation

---
## 2D LiDAR Mapping
We are going to use `Hector_Slam` to create a Map of our Gazebo environment with `LiDAR`
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

The RViz window will launch and you can start mapping by controlling the bot using `Teleop Keyboard Control`
- Launch Teleop Keyboard Control
  - Set Linear speed = 0.20
  - Set Angular speed = 0.25

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
---
## Move_Base Navigation
We will now implement `move_base` on our bot to navigate though our 2D Gazebo world map. We will use our preloaded `aruco_map.yaml` map

- Tech Stack of move_base
  - Global Costmap Path Planner: `Dijkstra algorithmn`
  - Local Costmap Path Planner: `DWA planner`

- Launch bot in Gazebo

``` bash
roslaunch criss_nav navros_aruco_control.launch
```

- Launch AMCL

```bash
roslaunch criss_nav amcl.launch
```

- Launch move_base

```bash
roslaunch criss_nav move_base.launch
```
- Visualize in RViz

```bash
cd criss_nav/rviz
rviz -d move_base.rviz
```
 You can now set `2D nav_goal` in RViz to visualize navigation.
 
![alt text](https://github.com/PranjalSri108/criss_nav/blob/main/move_base.jpg?raw=true)

---
## 3D Octomap 
We will now map a 3D Octomap of our environment using `Hector_Slam` and `Kinect Camera Plugin`
- But before launching the bot in gazebo, make sure the 'min' and 'max' params for Kinect plugin in `gazebo_plugins.urdf.xacro` are updated as given below

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
cd criss_nav/rviz
rviz -d octomap.rviz
```
You can now map the environment using the `Teleop Keyboard Control` with the same settings

![alt text](https://github.com/PranjalSri108/criss_nav/blob/main/octomap.jpg?raw=true)

---
## ArUco Marker Detection
We will now use `Kinect Camera Plugin` to detect the `ArUco Markers` present in our environment. 
We will then launch a Python script to publish the detected Marker ID to topic `/aruco_detect/id`

- Launch bot in Gazebo

``` bash
roslaunch criss_nav navros_aruco_control.launch
```

- Run Python script to detect and publish ArUco Id's

```bash
cd criss_nav/scripts
python detect_publish.py
```
You will now see the Kinect Camera input through `image_viewer`. you can now use `Teleop Keyboard Control` to move around to detect markers. Make sure that `/aruco_detect/id` is getting published.

![alt text](https://github.com/PranjalSri108/criss_nav/blob/main/aruco_detect.jpg?raw=true)

---
## ArUco based Navigation
Now we will navigate through the Gazebo enviroment by detecting the `ArUco Markers` along the way. We will run a Python script `nav.py` to do all this.

- Components of `nav.py`
  - Subscribes to the topics: `/aruco_detect/id` and `/odom`
  - Calculates the next `nav_goal` by using `/odom` data to estimate the bot's position
  - Applies Trignometry to calculate the coordinates of the next `nav_goal`
  - Sends to coordinates of nav_goal to the bot using `move_base_client`
  - Uses move_base to navigate.
 
- Work of `nav.py`
  - When detects a Marker ID 15: Turn the bot 90 degrees to the left and move straight until another marker is detected.
  - Used move_base to navigate through the environment.
  - When detects a Maker ID 16: Stops the bot

But before launching the bot in gazebo, make sure the 'min' and 'max' params for Kinect plugin in `gazebo_plugins.urdf.xacro` are updated as given below

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
                <far>2.5</far>
           </clip>
       </camera>
```

- Launch bot in Gazebo

``` bash
roslaunch criss_nav navros_aruco_control.launch
```

- Launch AMCL

```bash
roslaunch criss_nav amcl.launch
```

- Launch move_base

```bash
roslaunch criss_nav move_base.launch
```

- Run `nav.py` to initiate the `move_base_client`

```bash
cd criss_nav/scripts
python nav.py
```

- Run `detect_publish.py` to publish detected Marker ID
  
```bash
cd criss_nav/scripts
python detect_publish.py
```
You can observe the bot navigating in Gazebo using the ArUco Markers.

