#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

depth_scaled_img = np.zeros((360,480,3), np.uint16)
color_scaled_img = np.zeros(depth_scaled_img.shape, np.uint16)

class OpenRealsense:
	def __init__(self):
			self.bridge = CvBridge()
			self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback, queue_size=1)
			self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback, queue_size=1)

	def color_callback(self, data):
		global color_scaled_img
		try:
			color_scaled_img = self.bridge.imgmsg_to_cv2(data, "rgb8")
			color_scaled_img = cv2.cvtColor(color_scaled_img, cv2.COLOR_RGB2BGR)
			OpenRealsense.color_scaled_img = color_scaled_img
		except CvBridgeError as e:
			print(e)

	def depth_callback(self, data):
		global depth_scaled_img
		#print(data.encoding)
		#print(type(data)) # <class 'sensor_msgs.msg._Image.Image'>
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
			#print(type(cv_image)) # <type 'numpy.ndarray'>
			#print(cv_image.dtype)
			#print(cv_image.shape) # (360, 480)
			cv_image = np.array(cv_image, dtype=np.uint8)
		except CvBridgeError as e:
			print(e)

		min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv_image)

		if max_val == min_val:
			depth_scaled_img = cv_image
		else:
			depth_scaled_img = (cv_image - min_val) / (max_val - min_val) * 255.0

		# depth_scaled_img = np.zeros(cv_image.shape, dtype=cv_image.dtype)
		# cv2.normalize(cv_image, depth_scaled_img, 255.0, 0.0, cv2.NORM_MINMAX)

		# font = cv2.FONT_HERSHEY_SIMPLEX
		# tmpStr = "max={0} at {1}, min={2} at {3}".format(max_val, max_loc, min_val, min_loc)
		# cv2.putText(depth_scaled_img, tmpStr, (25,50), font, 0.4, (128,128,128), 2)

def main():
	global depth_scaled_img, refPt, cropping, color_scaled_img

	rospy.init_node('open_realsense_node', anonymous=True)
	pic = OpenRealsense()
	
	while True:
		cv2.imshow("Depth Image", depth_scaled_img)
		cv2.imshow("Color Image", color_scaled_img)
		
		key = cv2.waitKey(10) & 0xFF

		# if the 'x' key is pressed, break from the loop
		if key == ord('x'):
			break

	cv2.destroyAllWindows()

if __name__ == '__main__':
		main()