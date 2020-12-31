#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math
import matplotlib.pyplot as plt
import numpy as np

import sys

class lidar_alignment():
	def __init__(self):
		rospy.init_node('val2_planeAlignment_node', disable_signals=True)
		self.rotate_round = 0
		self.omeOld = 0
		self.planevel_publisher = rospy.Publisher('nav_vel', Twist, queue_size=10)
		self.listener()

	def listener(self):
		rospy.Subscriber('scan_filtered', LaserScan, self.callback, queue_size=1000)
		rospy.spin()

	def callback(self, data):
		scan_time = data.scan_time
		scan_time_increment = data.time_increment
		count = scan_time / scan_time_increment

		angle_min = self.rad2deg(data.angle_min)
		angle_max = self.rad2deg(data.angle_max)
		angle_increment = self.rad2deg(data.angle_increment)
		ranges = data.ranges

		i = 0
		coordinate_x = []
		coordinate_y = []

		while i < count:
			degree = angle_min + (angle_increment * i)
			if degree >= -10 and degree <= 10:
				if ranges[i] != float("inf"):
					# rospy.loginfo(": [%f, %f]", degree, ranges[i])
					x, y = self.polar2cartesian(ranges[i], degree)
					coordinate_x.append(x)
					coordinate_y.append(y)
			i = i + 1
	
		coordinate_x = np.array(coordinate_x)
		coordinate_y = np.array(coordinate_y)
		
		# print(len(coordinate_x), len(coordinate_y))
		if len(coordinate_x) < 20 and len(coordinate_y) < 20:
			rospy.signal_shutdown('Quit')

		elif len(coordinate_x) >= 20 and len(coordinate_y) >= 20:
			A = np.vstack([coordinate_x, np.ones(len(coordinate_x))]).T
			m, c = np.linalg.lstsq(A, coordinate_y, rcond=-1)[0]
		

		# print(m,c)
			
		# plt.cla()
		# plt.plot(coordinate_x, coordinate_y, 'o',
		# 		label='Original data', markersize=10)
		# plt.plot(coordinate_x, m*coordinate_x + c, 'r', label='Fitted line')
		# plt.ylabel('Range')
		# plt.xlabel('Degree')
		# plt.legend()
		# plt.pause(0.001)
					
		self.alignment_mode(m)

	def rad2deg(self, radians):
		pi = math.pi
		degrees = (180 * radians) / pi
		return degrees

	def deg2rad(self, degrees):
		pi = math.pi
		radians = degrees * (pi / 180)
		return radians

	def polar2cartesian(self, ranges, degrees):
		x = ranges * (math.sin(self.deg2rad(degrees)))
		y = ranges * (math.cos(self.deg2rad(degrees)))
		return x, y

	def alignment_mode(self, slope):
		print(slope)
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		if slope < -0.01:
			self.twist_robot.angular.z = 0.30
			if self.twist_robot.angular.z != self.omeOld :
				self.rotate_round = self.rotate_round + 1
			self.planevel_publisher.publish(self.twist_robot)
			print(self.rotate_round)
			self.omeOld = self.twist_robot.angular.z

		if slope > -0.01:
			self.twist_robot.angular.z = -0.30
			if self.twist_robot.angular.z != self.omeOld :
				self.rotate_round = self.rotate_round + 1
			self.planevel_publisher.publish(self.twist_robot)
			print(self.rotate_round)
			self.omeOld = self.twist_robot.angular.z

		if slope <= 0.005 and slope >= -0.005:
			print("stop")
			self.twist_robot.angular.z = 0
			self.planevel_publisher.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')

		if self.rotate_round >= 3:
			print("stop")
			self.twist_robot.angular.z = 0
			self.planevel_publisher.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')
	
if __name__ == '__main__':
	process = lidar_alignment()