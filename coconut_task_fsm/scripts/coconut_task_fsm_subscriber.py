#!/usr/bin/env python3

# Important Library
from std_msgs.msg import String
import rospy
import roslaunch
import os

def fsm_start():		
	fsm_node = roslaunch.core.Node(	package='coconut_task_fsm', 
									node_type='coconut_task_fsm.py', 
									name='coconut_task_fsm_node',
									output='screen')
	fsm_launch = roslaunch.scriptapi.ROSLaunch()
	fsm_launch.start()
	fsm_process = fsm_launch.launch(fsm_node)
	while fsm_process.is_alive():
		if fsm_process.is_alive() == False:
			break

class gui_cmd_subscriber(object):

	def __init__(self):
		self.gui_command = None

	def callback(self,data):
		self.gui_command = data.data

	def listener(self):
		rospy.Subscriber('gui_cmd', String, self.callback)


if __name__=="__main__":
	rospy.init_node('coconut_task_fsm_subscriber_node')
	cmdSub_node = gui_cmd_subscriber()
	cmdSub_node.listener()
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = cmdSub_node.gui_command
			if command == "start":
				fsm_start()
				cmdSub_node.gui_command = None
			r.sleep()
	except Exception as e:
		print(e)
