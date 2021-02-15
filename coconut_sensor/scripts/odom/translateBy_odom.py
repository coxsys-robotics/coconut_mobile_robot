#!/usr/bin/env python3

# Import necessary package
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import sys
from tf.transformations import euler_from_quaternion
 
# Class for sending velocity command to base controller
class send_cmdvel():

	# Initial state
	def __init__(self):
		# Publisher configuration
		self.pub = rospy.Publisher('/nav_vel', Twist, queue_size=1)
		# Initialize target distance (cm)
		self.target = rospy.get_param("~translate_target", 100) #cm
		# Time variation variable 
		self.diff = 0
		self.filter_pose = 0
		self.kp = 0.1
		self.setPose = True
		self.velocity = 0
		self.ramp_up = True
		self.ramped_vel = 0.05
		# Define offset
		self.offset = 7
		# Initialize velocity command
		self.twist_robot =Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0

	# Select state depend on input distance
	def selectState(self):
		if self.target >= 0:
			self.state = "move_forward"
		elif self.target < 0:
			self.state = "move_backward"

	# Main function make robot moving forward
	def moveRobot(self, state, robot_pose, robot_yaw):

		if state == "move_forward":
			if self.setPose == True:
				self.first_pose = robot_pose
				self.target = self.target * abs(math.cos(robot_yaw))
				self.setPose = False

			self.filter_pose = abs(robot_pose - self.first_pose)

			self.diff = abs(self.target - self.filter_pose)
			self.velocity = (self.kp * (self.diff) * 0.25) / (self.target * self.kp)
			if self.ramp_up == True:
				self.ramped_vel = self.ramped_vel + 0.01
				if self.ramped_vel >= 0.25:
					self.ramp_up = False
				self.velocity = self.ramped_vel
			elif self.velocity <= 0.18:
				self.velocity = 0.18
			self.twist_robot.linear.x = self.velocity
			self.pub.publish(self.twist_robot)
			if abs(self.diff) < self.offset:
				state = "stop"

			print("\n")
			print("Robot state={}".format(state))
			print("Robot velocity={}".format(self.velocity))
			print("target={} current={}".format(self.target, self.filter_pose))
		
		if state == "stop":
			self.twist_robot.linear.x = 0
			self.pub.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')
		  
# Class for receive the odometry from odom node
class odom_subscriber(object):

	# Initial state
	def __init__(self):
		self.pose_centimeter = 0.0
		self.angle_rad = 0.0

	# Storage odometry data
	def get_position(self, msg):
		position_q = msg.pose.pose.position
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.angle_rad = yaw #rad
		self.pose_centimeter = position_q.x * 100 #cm

	# Call get_position function when subscribe to 'odom' topic in Odometry type
	def listener(self):
		rospy.Subscriber ('/odom', Odometry, self.get_position)


if __name__=="__main__":
	# Define node name
	rospy.init_node('translateBy_odom_node', disable_signals=True)

	# Define necessary variable which calling each class
	odomSub_node = odom_subscriber()
	odomSub_node.listener()
	sendCmdvel_node = send_cmdvel()
	sendCmdvel_node.selectState()
	state = sendCmdvel_node.state
	# Initialize first yaw
	first_yaw = True
	yaw = 0
	# Loop frequency
	r = rospy.Rate(10)
	rospy.sleep(1)
	try:
		while(not rospy.is_shutdown()):
			if first_yaw == True:
				yaw = odomSub_node.angle_rad
				first_yaw = False
			sendCmdvel_node.moveRobot(state, odomSub_node.pose_centimeter, yaw)
			r.sleep()
	except Exception as e:
		print(e)
