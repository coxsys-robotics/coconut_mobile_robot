#!/usr/bin/env python3

# Warning for 2 times then robot find new path 
 
# Important necessary package
import os
import math
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from switch_obstacleAvoidance import open_obstacleAvoidance, close_obstacleAvoidance


# Class for detect the obstacle 
class obstacle_detection():

	# Initial state
	def __init__(self):
		# Define node name
		rospy.init_node('coconut_obstacleDetection_type2_node', disable_signals=True)
		# Publisher configuration
		self.cmdvel_publisher = rospy.Publisher('break_vel', Twist, queue_size=10)
		self.emer_publisher = rospy.Publisher('alert', String, queue_size=10)
		# Time invariation variable
		self.scan_range = 0.8 #m 
		self.scan_fov = 0.8 #m 
		self.first_time = True
		# Time variation variable
		self.start_time = 0.0
		self.interval_time = 0.0
		self.waiting_time = 16.0 # Afthe end sound wait for 16 - 6(Sound) second then open the obstacle avoidance
		self.first_obstacle_time = True
		self.start_obstacle_time = 0.0
		self.interval_obstacle_time = 0.0
		self.waiting_obstacle_time = 10.0 # Wait for 10 second then close the obstacle avoidance
		self.ready4avoidance = False
		self.emer_round = 0
		self.interest_param_global = '/move_base/global_costmap/obstacle_layer/enabled'
		self.interest_param_local = '/move_base/local_costmap/obstacle_layer/enabled'
		self.params = os.popen("rosparam list").read().splitlines()
		self.set_param = True
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
		
		if self.set_param == True:
			rospy.set_param("break_cond", False)
			self.set_param = False

		if obstacle == True and self.ready4avoidance == True:
			print("Continue moving")
			if self.interest_param_global in self.params and self.interest_param_local in self.params:
				enable_global = rospy.get_param(self.interest_param_global, None)
				enable_local = rospy.get_param(self.interest_param_local, None)
				if enable_global == False and enable_local == False:
					open_obstacleAvoidance()
				else:
					pass

		elif obstacle == True and self.interval_time < self.waiting_time:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			if self.first_time == True:
				self.start_time = time.time()
				if self.emer_round < 1:
					self.emer_publisher.publish("True")
					self.emer_round = self.emer_round + 1
				self.first_time = False
			self.interval_time = time.time() - self.start_time
			self.first_obstacle_time = True

		elif obstacle == True and self.interval_time >= self.waiting_time:
			# Incase want to have more than one time emergency
			# if self.emer_round < 2:
			# 	print("Stop")
			# 	self.twist_robot.linear.x = 0
			# 	self.twist_robot.angular.z = 0
			# 	self.cmdvel_publisher.publish(self.twist_robot)
			# 	self.first_time = True
			# 	self.interval_time = 0
			# elif self.emer_round == 2:
			# 	self.ready4avoidance = True

			# Incase want to have only one time emergency
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			self.first_time = True
			self.interval_time = 0
			self.ready4avoidance = True

		elif obstacle == False:
			print("Continue moving")
			self.first_time = True
			self.interval_time = 0
			self.emer_round = 0
			if self.first_obstacle_time == True:
				self.start_obstacle_time = time.time()
				self.first_obstacle_time = False

			self.interval_obstacle_time = time.time() - self.start_obstacle_time
			
			if self.interest_param_global in self.params and self.interest_param_local in self.params:
				enable_global = rospy.get_param(self.interest_param_global, None)
				enable_local = rospy.get_param(self.interest_param_local, None)
				if enable_global == True and enable_local == True:
					if self.interval_obstacle_time > self.waiting_obstacle_time:
						close_obstacleAvoidance()
						self.interval_obstacle_time = 0
						self.first_obstacle_time = True
						self.ready4avoidance = False
				else:
					pass
		
	#Convert angle from radians to degree format
	def rad2deg(self, radians):
		pi = math.pi
		degrees = (180 * radians) / pi
		return degrees


if __name__ == '__main__':
	try:
		process = obstacle_detection()
	except Exception as e:
		print(e)
	finally:
		interest_param_global = '/move_base/global_costmap/obstacle_layer/enabled'
		interest_param_local = '/move_base/local_costmap/obstacle_layer/enabled'
		enable_global = rospy.get_param(interest_param_global, None)
		enable_local = rospy.get_param(interest_param_local, None)
		if enable_global == True and enable_local == True:
			close_obstacleAvoidance()