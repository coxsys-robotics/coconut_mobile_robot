#!/bin/bash

### to run this file on robot startup using crontab
##
## $ crontab -e
##
## will edit crontab file, then place this command
##
## @reboot <directort to startup.sh>
#
# e.g. @reboot /home/coconut/coconut_ws/src/coconut_mobile_robot/Startup.sh
#

### localhost ###
#export ROS_MASTER_URI=http://localhost:11311
#export ROS_HOSTNAME=localhost
### office wifi ###
#export ROS_MASTER_URI=http://192.168.2.109:11311
#export ROS_HOSTNAME=192.168.2.109
### LAN ###
#export ROS_MASTER_URI=http://10.0.0.60:11311
#export ROS_HOSTNAME=10.0.0.60
### Hotspot ###
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_HOSTNAME=10.42.0.1

### melodic ###
# source /opt/ros/melodic/setup.bash
### noetic ###
source /opt/ros/noetic/setup.bash

### directory to ros workspace
source /home/coconut/coconut_ws/devel/setup.bash

### Path to .launch file
export launch_path="/home/coconut/coconut_ws/src/coconut_mobile_robot/coconut_robot/launch/call_launch.launch"

### topic name when robot shutdown. core_bringup_manager.py will shutdown ros then robot after receive this specific topic
export shutdown_topic="powerboard_status"

/opt/ros/noetic/bin/rosrun coconut_control core_bringup_manager.py "$launch_path" "$shutdown_topic"
