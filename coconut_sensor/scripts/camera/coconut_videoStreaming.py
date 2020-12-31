#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import cv2
import datetime
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imagezmq.imagezmq import ImageSender

color_scaled_img = np.zeros((640,480,3), np.uint16)

class OpenRealsense:
	def __init__(self):
			self.bridge = CvBridge()
			self.sender = ImageSender(connect_to='tcp://192.168.2.140:5555')
			self.camera_name = "realsense_camera"
			self.jpeg_quality = 50
			self.listener()
			
	def color_callback(self, data):
		try:
			color_scaled_img = self.bridge.imgmsg_to_cv2(data, "rgb8")
			self.color_scaled_img = cv2.cvtColor(color_scaled_img, cv2.COLOR_RGB2BGR)
			image = self.color_scaled_img
			timestamp = datetime.datetime.now()
			cv2.putText(image, timestamp.strftime("%A %d %B %Y %I:%M:%S%p"), (10, image.shape[0] - 10),
						cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
			ret_code, jpg_buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
			self.sender.send_jpg(self.camera_name, jpg_buffer)
		except CvBridgeError as e:
			print(e)

	def listener(self):
		self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback, queue_size=1)
		rospy.spin()


if __name__ == '__main__':
	rospy.init_node('coconut_videoStreaming_node', disable_signals=True)
	process = OpenRealsense()