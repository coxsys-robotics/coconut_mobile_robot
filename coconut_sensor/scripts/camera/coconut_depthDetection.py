#!/usr/bin/env python

from __future__ import print_function

import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2


class depthTracking():
	def __init__(self):
		rospy.init_node('coconut_depthDetection_node', disable_signals=True)
		self.depthvel_publisher = rospy.Publisher('camera_depth_detection', Twist, queue_size=10)
		self.depth_data = None
		self.target_distance = rospy.get_param("~distance_target", 500.0)
		self.bridge = CvBridge()
		self.depth_listener()

	def depth_listener(self):
		depth_image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_image_callback, queue_size=1)
		rospy.spin()

	def depth_image_callback(self, data):
		try:
			# width = 640, height = 480
			cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

			# # Single Depth Measurement
			pix = (data.width/2, data.height/2) 
			x = pix[0]
			y = pix[1]
			depth_data = cv_image[y, x]
			self.depth_data = depth_data
			obstacle_check = False
			print('Depth at center: {}(mm)\r'.format(self.depth_data))
			if self.depth_data >= self.target_distance:
				obstacle_check = False			
			elif self.depth_data != 0 and self.depth_data < self.target_distance:
				obstacle_check = True

			# Multi Depth Measurement
			# obstacle_check = False
			# for y in range(214, 370):
			# 	for x in range(213, 426):
			# 		depth_data = cv_image[y, x]			
			# 		if depth_data != 0 and depth_data < self.target_distance:
			# 			obstacle_check = True
			# 			break
			# 	else:
			# 		continue
			# 	break
			# cv_image = np.array(cv_image, dtype=np.uint8)

			self.check_mode(obstacle_check)
		except CvBridgeError as e:
			print(e)

	def check_mode(self, obstacle):
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0

		if obstacle == True:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.depthvel_publisher.publish(self.twist_robot)
		elif obstacle == False:
			print("Continue moving")
			pass
	

if __name__ == '__main__':
	process = depthTracking()