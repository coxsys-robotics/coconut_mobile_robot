#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray, UInt8
# from geometry_msgs.msg import Twist


class ultrasonicData_subscriber(object):
	
	def __init__(self):
		rospy.init_node('ultrasonic_subscriber_node', anonymous=True)
		# self.pub_robot_velocity = rospy.Publisher('ultrasonic_detected', Twist, queue_size = 10)  

		self.ultrasonic_data = None
		# self.ultrasonic_detected = None

		self.max_range = 60 # centimetre
		self.min_range = 40 # centimetre
		self.min_ult = 20
		self.max_factor = 1.0
		self.min_factor = 0.9
		self.previous_back_factor = 1
		self.previous_front_factor = 1

		self.use_bumper = True
		self.bumper_status = 0 ## not_bump:0, front:1, back:2
		self.bumper_status_sub = rospy.Subscriber('bumper_detected', UInt8, self.bumper_callback)

		# self.twist_robot = Twist()
		# self.twist_robot.linear.x = 0
		# self.twist_robot.linear.y = 0
		# self.twist_robot.linear.z = 0
		# self.twist_robot.angular.x = 0
		# self.twist_robot.angular.y = 0
		# self.twist_robot.angular.z = 0

		self.listener()



	"""        ROBOT
                                        Front Robot
                                                ^
                                                |
                       				__________________________
                      	/         0        1         2        3         \
                      |                                                   |
                      |                                                   |
                      |                                                   |
                      |                                                   |
                      |                        O                         |
                      |                                                   |
                      |                                                   |
                      |                                                   |
                      	 \9                                           4/
                       		  \-------8------7-----6------5-------/
                               
                                       Top view
    """ 

	"""
	forward factor
			
			|				.		.
			|				.		.
			|				.		.
  max_factor|				.		- - - - - - - - -
			|				.	   /.
			|				.	  /	.
			|				.	 /	.	
			|				.	/	.
			|				.  /	.
			|				. /		.
			|				./		.
  min_factor|		 . - - - -		.
			|		 .		.		.
			|		 .		.		.
			|_________________________________________________ distance
			0		 v		v		v
				  min_ult	v		
						min_range	v	
								max_range
	"""


	def ultrasonic_callback(self,data):
		### 2 Straight Front Ultrasonics ### (ultrasonic no.1 and no.2)
		if( min(data.data[1:3]) <= self.min_ult ):
			rospy.set_param("/config/forward_factor", 0.0)

		### All Front ultrasonics ###
		else:
			min_front = min( data.data[0:4] )
			if(min_front >= self.max_range):
				factor_to_set = 1
			elif(min_front <= self.min_range):
				factor_to_set = self.min_factor
			else:
				factor_to_set = (min_front * (1 - self.min_factor) / (self.max_range - self.min_range)) + ( 1 - self.max_range * (self.max_factor - self.min_factor) / (self.max_range - self.min_range) )
				# factor_to_set *= self.previous_front_factor
				# if(factor_to_set < self.min_factor):
				# 	factor_to_set = self.min_factor
			if(self.use_bumper):
				# self.previous_front_factor = factor_to_set
				if(self.bumper_status!=1):
					rospy.set_param("/config/forward_factor", factor_to_set)
				else:
					rospy.set_param("/config/forward_factor", 0.0)

		### 2 Straight Back ultrasonics ###
		if( min(data.data[5:9]) <= self.min_ult ):
			rospy.set_param("/config/backward_factor", 0.0)

		### All Back ultrasonics ###
		else:
			min_back = min( data.data[4:10] )
			if(min_back >= self.max_range):
				factor_to_set = 1
			elif(min_back <= self.min_range):
				factor_to_set = self.min_factor
			else:
				factor_to_set = (min_back * (1 - self.min_factor) / (self.max_range - self.min_range)) + ( 1 - self.max_range * (self.max_factor - self.min_factor) / (self.max_range - self.min_range) )
				# factor_to_set *= self.previous_back_factor
				# if(factor_to_set < self.min_factor):
				# 	factor_to_set = self.min_factor
			if(self.use_bumper):
				# self.previous_back_factor = factor_to_set
				if(self.bumper_status!=2):
					rospy.set_param("/config/backward_factor", factor_to_set)
				else:
					rospy.set_param("/config/backward_factor", 0.0)


	def bumper_callback(self,data):
		self.bumper_status = data.data

	# def publish_velocity(self, ultrasonic_data):
	# 	pass
		# print(ultrasonic_data.data)
		# rate = rospy.Rate(10)
		# if 1 in ultrasonic_data.data:
		# 	self.ultrasonic_detected = True
		# 	while(self.ultrasonic_detected == True):
		# 		self.pub_robot_velocity.publish(self.twist_robot)
		# 		rate.sleep()
		# elif 1 not in ultrasonic_data.data:
		# 	self.ultrasonic_detected = False

	def listener(self):
		rospy.Subscriber('ultrasonic_data', UInt16MultiArray, self.ultrasonic_callback)
		rospy.spin()


if __name__=="__main__":
	try:
		process = ultrasonicData_subscriber()
	except Exception as e:
		print(e)