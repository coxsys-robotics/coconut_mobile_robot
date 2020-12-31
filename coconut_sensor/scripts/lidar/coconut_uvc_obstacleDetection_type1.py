#!/usr/bin/env python

# Wait until obstacle give a way

# Important necessary package
import os
import math
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Class for detect the obstacle 
class obstacle_detection():

	# Initial state
	def __init__(self):
		# Define node name
		rospy.init_node('coconut_obstacleDetection_type1_node', disable_signals=True)
		# Publisher configuration
		self.cmdvel_publisher = rospy.Publisher('break_vel', Twist, queue_size=10)
		self.emer_publisher = rospy.Publisher('alert', String, queue_size=10)
		# Time invariation variable
		self.first_time = True
		self.scan_range = 0.8 #m 
		self.scan_fov = 0.8 #m
		# Time variation variable
		self.start_time = 0.0
		self.interval_time = 0.0
		self.waiting_time = 10.0
		self.emer_round = 0
		# Call function
		self.find_interestAngle()
		self.listener()

	# Define the detection area 
	def find_interestAngle(self):
		anotherAngle = math.atan((self.scan_fov/2)/self.scan_range)
		anotherAngle = self.rad2deg(anotherAngle)
		self.interestAngle = anotherAngle

	# Call call back function when subscribe to 'scan' topic in LaserScan type 
	def listener(self):
		rospy.Subscriber('scan', LaserScan, self.callback, queue_size=1000)
		rospy.spin()

	# Computing the data from lidar and select the robot state
	def callback(self, data):
		scan_time = data.scan_time
		scan_time_increment = data.time_increment
		count = scan_time / scan_time_increment

		angle_min = self.rad2deg(data.angle_min)
		angle_max = self.rad2deg(data.angle_max)
		angle_increment = self.rad2deg(data.angle_increment)
		ranges = data.ranges

		i = 0 
		obstacle_range = []
		obstacle_check = False
		 
		while i < count:
			degree = angle_min + (angle_increment * i)
			if degree >= -self.interestAngle and degree <= self.interestAngle:
				if ranges[i] > self.scan_range:
					# rospy.loginfo(": [%f, %f]", degree, ranges[i])
					obstacle_check = False
				elif ranges[i] <= self.scan_range:
					# rospy.loginfo(": [%f, %f]", degree, ranges[i])
					obstacle_check = True
					break
			i = i + 1
		self.check_mode(obstacle_check)

	# Check robot state
	def check_mode(self, obstacle):
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		
		if obstacle == True and self.interval_time < self.waiting_time:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			if self.first_time == True:
				self.start_time = time.time()
				if self.emer_round < 2:
					self.emer_publisher.publish("True")
					self.emer_round = self.emer_round + 1
				self.first_time = False
			self.interval_time = time.time() - self.start_time

		elif obstacle == True and self.emer_round >=2:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)

		elif obstacle == True and self.interval_time >= self.waiting_time:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			
			self.first_time = True
			self.interval_time = 0

		elif obstacle == False:
			print("Continue moving")
			self.first_time = True
			self.interval_time = 0
			self.emer_round = 0
			pass
	
	#Convert angle from radians to degree format
	def rad2deg(self, radians):
		pi = math.pi
		degrees = (180 * radians) / pi
		return degrees


if __name__ == '__main__':
	process = obstacle_detection()
