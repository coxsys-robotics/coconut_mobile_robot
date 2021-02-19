#!/usr/bin/env python3

# Important Library
import os
import rospy
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def open_pause_subscriber_node():
    pause_node = roslaunch.core.Node(package='coconut_task_fsm', 
                                     node_type='coconut_task_pause_subscriber.py', 
                                     name='coconut_task_pause_subscriber_node',
                                     output='screen')
    pause_launch = roslaunch.scriptapi.ROSLaunch()
    pause_launch.start()
    pause_process = pause_launch.launch(pause_node)
    while pause_process.is_alive():
        if pause_process.is_alive() == False:
            break


class gui_cmd_subscriber(object):

	def __init__(self):
		self.gui_command = None

	def callback(self,data):
		self.gui_command = data.data

	def listener(self):
		rospy.Subscriber('gui_cmd', String, self.callback)


if __name__=="__main__":
	rospy.init_node('coconut_task_continue_subscriber_node')
	cmdSub_node = gui_cmd_subscriber()
	cmdSub_node.listener()
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = cmdSub_node.gui_command
			if command == "continue":
				open_pause_subscriber_node()
			r.sleep()
	except Exception as e:
		print(e)