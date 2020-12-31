#!/usr/bin/env python

import time
import rospy
import actionlib
from math import sqrt
from os.path import expanduser
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

home = expanduser("~")


class cancel_movebase_goal():
	def __init__(self):
		rospy.init_node('cancel_goal_node', anonymous=True)

		costmap_path = "{}/coconut_ws/src/coconut_bringup/teb_config/".format(home)
		costmap_filename = "local_costmap_params.yaml"
		costmap_file = costmap_path + costmap_filename

		self.offset = 0
		self.stuck_time = 10
		self.robot_half_length = 0.326
		self.inflation_radius = self.read_inflation_radius(costmap_file)

		if self.robot_half_length < self.inflation_radius:
			self.offset = self.inflation_radius + 0.1
		elif self.robot_half_length >= self.inflation_radius:
			self.offset = self.robot_half_length + 0.1
		print(self.offset)

		self.start_time = None
		self.end_time = None
		self.interval_time = 0
		self.first_time = True

		self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server...")
		# Wait 60 seconds for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(60))
		rospy.loginfo("Connected to move base server")
		rospy.loginfo("Starting cancel move_base goal node")

		rospy.Subscriber("pose_from_robot", Pose, self.odomCallback)
		rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goalCallback)

		rospy.spin()

	def read_inflation_radius(self, filename):
		inflation_radius = 0
		file = open(filename)
		word = "inflation_radius"
		for line in file:
			line.strip().split('/n')
			if word in line:
				inflation_radius=line[line.find(":")+1:]
		inflation_radius = float(inflation_radius.strip(' '))
		
		return inflation_radius

	def odomCallback(self, msg):
		self.robot_pose = msg

	def goalCallback(self, msg):
		self.goal_pose = msg.pose
		while True:
			# print("robot pose: {}, {}".format(self.robot_pose.position.x, self.robot_pose.position.y))
			# print("goal pose: {}, {}".format(self.goal_pose.position.x, self.goal_pose.position.y))
			self.distance = sqrt(pow(self.goal_pose.position.x - self.robot_pose.position.x, 2) + pow(self.goal_pose.position.y - self.robot_pose.position.y, 2))
			print(self.distance)

			if self.first_time == False:
				self.end_time = time.time()
			
			if self.distance < self.offset:
				if self.first_time == True:
					self.start_time = time.time()
					self.first_time = False

			if self.first_time != None and self.end_time != None:
				self.interval_time = abs(self.end_time - self.start_time)
				print("TIME START: {}".format(self.start_time))
				print("END TIME: {}".format(self.end_time))
				print("INTERVAL TIME: {}".format(self.interval_time))

			if self.interval_time >= self.stuck_time:
				cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
				cancel_msg = GoalID()
				cancel_pub.publish(cancel_msg)
				rospy.sleep(1)
				rospy.loginfo("goal has been canceled")
				break


if __name__ == "__main__":
	cancel_movebase_goal()