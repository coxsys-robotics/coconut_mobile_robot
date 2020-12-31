#!/usr/bin/env python
from __future__ import print_function

import cv2
# import sys
# import rospy

class OpenIPCAM:

	def __init__(self):
		self.user = ""
		self.password = ""
	  	self.cap = cv2.VideoCapture('rtsp://{}:{}!@192.168.1.106/1'.format(self.user, self.password))
		while(True):
			# Capture frame-by-frame
			ret, frame = self.cap.read()

			# Our operations on the frame come here
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# Display the resulting frame
			cv2.imshow('frame',gray)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		self.cap.release()
		cv2.destroyAllWindows()

def main():
	ic = OpenIPCAM()
	rospy.init_node('open_ipcam_node', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	process = main()