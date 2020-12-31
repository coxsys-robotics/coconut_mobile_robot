#!/usr/bin/env python

# Import necessary package 
import rospy
import actionlib
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

"""
Subscribe to /joy and send command to cancel movebase_goal
"""
class joycmd_subscriber(object):

	def __init__(self):
		# Define node name
		rospy.init_node('skipGoal_byJoy_node')

		self.joy_command = None
		
		self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server...")
		self.move_base.wait_for_server(rospy.Duration(60))
		rospy.loginfo("Connected to move base server")
		rospy.loginfo("Starting move base goals smoother")
		
		self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
		
		self.joycmd_listener()

	def callback(self, data):
		self.joy_command = data.buttons
		self.toggled()

	def toggled(self):
		if self.joy_command[4] == 1:
			while True:
				cancel_msg = GoalID()
				self.cancel_pub.publish(cancel_msg)
				rospy.sleep(1)
				rospy.loginfo("goal has been canceled")
				break


	def joycmd_listener(self):
		rospy.Subscriber('joy', Joy, self.callback)
		rospy.spin()


if __name__=="__main__":

	process = joycmd_subscriber()