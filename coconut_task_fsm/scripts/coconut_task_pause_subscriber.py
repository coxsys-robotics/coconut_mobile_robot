#!/usr/bin/env python

# Important Library

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy
import roslaunch
import os


class pause_vel_publisher():

	def __init__(self):
		self.pub_robot_velocity = rospy.Publisher('paused_vel', Twist, queue_size = 10)  
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0
		
	def publish_velocity(self):
		rate = rospy.Rate(10)
		while(not rospy.is_shutdown()):
			self.pub_robot_velocity.publish(self.twist_robot)
			rate.sleep()


class gui_cmd_subscriber(object):

	def __init__(self):
		self.gui_command = None

	def callback(self,data):
		self.gui_command = data.data

	def listener(self):
		rospy.Subscriber('gui_cmd', String, self.callback)


if __name__=="__main__":
	rospy.init_node('coconut_task_pause_subscriber_node')
	cmdSub_node = gui_cmd_subscriber()
	pauseVel_node = pause_vel_publisher()
	cmdSub_node.listener()
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = cmdSub_node.gui_command
			if command == "pause":
				pauseVel_node.publish_velocity()
			r.sleep()
	except Exception as e:
		print(e)