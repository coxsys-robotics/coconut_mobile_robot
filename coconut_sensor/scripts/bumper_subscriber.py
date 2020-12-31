#!/usr/bin/env python

import rospy
from std_msgs.msg import ByteMultiArray, UInt8
# from geometry_msgs.msg import Twist

"""
Subscribe to bumper sensor. Set movement factor accordingly.
"""

class bumperData_subscriber(object):
	
	def __init__(self):
		rospy.init_node('bumper_subscriber_node', anonymous=True)

		### publish bumper status - none:0, front:1, back:2
		self.pub_bumper_status = rospy.Publisher('bumper_detected', UInt8, queue_size = 10)  

		self.bumper_data = None
		# self.bumper_detected = None

		# self.twist_robot = Twist()
		# self.twist_robot.linear.x = 0
		# self.twist_robot.linear.y = 0
		# self.twist_robot.linear.z = 0
		# self.twist_robot.angular.x = 0
		# self.twist_robot.angular.y = 0
		# self.twist_robot.angular.z = 0

		self.listener()
		
	# def publish_velocity(self, bumper_data):
	# 	rate = rospy.Rate(10)
	# 	if 1 in bumper_data.data:
	# 		self.bumper_detected = True
	# 		while(self.bumper_detected == True):
	# 			# print("STOP")
	# 			self.pub_robot_velocity.publish(self.twist_robot)
	# 			rate.sleep()
	# 	elif 1 not in bumper_data.data:
	# 		self.bumper_detected = False
	# 		# print("CONTINUE")

	def bumper_callback(self, data):
		msg = UInt8()
		### Bumper controling linear movement ###
		## Front bumpers. make forward factor = 0
		if(data.data[0] or data.data[1] or data.data[2]):  #(data.data & 0b000111): 
			rospy.set_param("/config/forward_factor", 0.0)
			rospy.set_param("/config/backward_factor", 1.0)
			msg.data = 1
		## Back bumpers. make backward_factor = 0
		elif(data.data[3] or data.data[4] or data.data[5]): 
			rospy.set_param("/config/forward_factor", 1.0)
			rospy.set_param("/config/backward_factor", 0.0)
			msg.data = 2
		## No bumper detected. can move normally
		else:
			rospy.set_param("/config/forward_factor", 1.0)
			rospy.set_param("/config/backward_factor", 1.0)
			msg.data = 0
		self.pub_bumper_status.publish(msg)
		###   ###   ###

		### Bumper controling angular movement ###
		if(data.data[1] or data.data[4]):
			rospy.set_param("/config/spin_factor", 0.0)
		else:
			rospy.set_param("/config/spin_factor", 1.0)


	def listener(self):
		rospy.Subscriber('bumper_data', ByteMultiArray, self.bumper_callback)
		rospy.spin()


if __name__=="__main__":
	try:
		process = bumperData_subscriber()
	except Exception as e:
		print(e)