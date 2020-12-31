#!/usr/bin/env python

import rospy

import tf

from geometry_msgs.msg import Twist
import time

class send_cmdvel():

	def __init__(self):
		self.pub = rospy.Publisher('/nav_vel', Twist, queue_size=1)
		self.target = rospy.get_param("~translate_target", 200) #cm

		self.diff = 0
		self.kp = 0.1

		self.twist_robot =Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0

		self.VL = 20 #cm/s 
		self.VR = 20 #cm/s
		
		self.v_rx = 0.0
		self.pose_x = 0.0
		
		self.start_time = 0
		self.first_time = True


	def selectState(self):
		if self.target > 0:
			self.state = "move_forward"
		elif self.target < 0:
			self.state = "move_backward"
		elif self.target == 0:
			self.state = "stop"


	def compute_distance(self):
		if self.first_time == True:
			self.start_time = time.time()
			self.first_time = False
		elapsed_time = time.time() - self.start_time
		robot_linear_velocity = (self.VL + self.VR)/2

		self.v_rx = robot_linear_velocity
		self.pose_x = self.v_rx * elapsed_time


	def rotateRobot(self, state):
		if state == "move_forward":
			actual_pose = self.pose_x
			target_pose = self.target
			self.diff = abs(target_pose - actual_pose)

			velocity = self.kp * self.diff
			if velocity >= self.VL/100.0:
				velocity = self.VL/100.0
			self.twist_robot.linear.x = velocity
			self.pub.publish(self.twist_robot)
			if abs(self.diff) < 1:
				state = "stop"

			print("\n")
			print("Robot state={}".format(state))
			print("Robot velocity={}".format(velocity))
			print("target={} current={}".format(target_pose, actual_pose))

			self.compute_distance()
		
		if state == "move_backward":
			actual_pose = self.pose_x
			target_pose = self.target
			self.diff = abs(target_pose + actual_pose)

			velocity = self.kp * self.diff
			if velocity >= -self.VL/100.0:
				velocity = -self.VL/100.0
			self.twist_robot.linear.x = velocity
			self.pub.publish(self.twist_robot)
			if abs(self.diff) < 1:
				state = "stop"

			print("\n")
			print("Robot state={}".format(state))
			print("Robot velocity={}".format(-velocity))
			print("target={} current={}".format(target_pose, -actual_pose))

			self.compute_distance()

		if state == "stop":
			self.twist_robot.linear.x = 0
			self.pub.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')


if __name__=="__main__":
	rospy.init_node('translateBy_timer_node', disable_signals=True)

	sendCmdvel_node = send_cmdvel()
	sendCmdvel_node.selectState()
	state = sendCmdvel_node.state
	
	r = rospy.Rate(10)
	rospy.sleep(1)
	try:
		while(not rospy.is_shutdown()):
			sendCmdvel_node.rotateRobot(state)
			r.sleep()
	except Exception as e:
		print(e)