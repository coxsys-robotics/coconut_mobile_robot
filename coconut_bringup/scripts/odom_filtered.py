#!/usr/bin/env python3

# Import necessary package 
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

"""
transform odom frame
"""
class t265_odom_subscriber(object):

	def __init__(self):
		rospy.init_node('odom_filtered_node')
		self.odom_pub = rospy.Publisher("coconut_odometry", Odometry, queue_size=1)
		self.t265_odom_listener()

	"""
	rotate odom 180 degree around z axis because t265 +x is robot's -x axis and +z axis is verticle to the floor.
	"""
	def callback(self, data):
		x_filtered = data.pose.pose.position.x * -1
		y_filtered = data.pose.pose.position.y * -1
		
		orientation_z = data.pose.pose.orientation.z
		orientation_w = data.pose.pose.orientation.w

		coconut_odometry = Odometry()
		coconut_odometry.pose.pose.position.x = x_filtered
		coconut_odometry.pose.pose.position.y = y_filtered
		coconut_odometry.pose.pose.orientation.z = orientation_z
		coconut_odometry.pose.pose.orientation.w = orientation_w
		
		coconut_odometry.header.stamp = rospy.Time.now()
		coconut_odometry.header.frame_id = data.header.frame_id
		coconut_odometry.child_frame_id = data.child_frame_id

		self.odom_pub.publish(coconut_odometry)
		
		# orientation_list = [0.0, 0.0, orientation_z, orientation_w]
		# (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

		# self.rad2deg(yaw)
		# print("roll = {}, pitch = {}, yaw = {}, degree = {}".format(roll, pitch, yaw, self.degrees))

		# print(x_filtered, y_filtered)

	def rad2deg(self, radians):
		pi = math.pi
		self.degrees = (180 * radians) / pi

	def t265_odom_listener(self):
		rospy.Subscriber('/t265/odom/sample', Odometry, self.callback)
		rospy.spin()


if __name__=="__main__":
	process = t265_odom_subscriber()