# Fusion of Robotic Exploration and Object Detection: Search and Rescue

## Teaming Information
Project Team 5: Wil Carrasco, Tom Casaletto, Alex Dodd


To run Lidar camera fusion

roslaunch transbot_nav transbot_bringup.launch
roslaunch darknet_ros darknet_ros_nodelet.launch
roslaunch transbot_nav  transbot_map.launch map_type:=gmapping
OR 
roslaunch transbot_nav rrt_exploration.launch
rosrun laser_camera_fusion laser_reader.py

## Project Description

## Project Idea

## Proposed Solution
