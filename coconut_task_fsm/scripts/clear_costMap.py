#!/usr/bin/env python3

# Import necessary package
import rospy
import roslaunch

# Function for clear cost map in navigation system
def clear_costmaps():
	rospy.loginfo('Executing state clear costmaps')
	clear_costmaps_node = roslaunch.core.Node(package='coconut_task_fsm', 
											  node_type='clear_costMap.py', 
											  name='clear_costmap_node')
	clear_costmaps_launch = roslaunch.scriptapi.ROSLaunch()
	clear_costmaps_launch.start()
	clear_costmaps_process = clear_costmaps_launch.launch(clear_costmaps_node)
	while clear_costmaps_process.is_alive():
		if clear_costmaps_process.is_alive() == False:
			break
	clear_costmaps_process.stop()
