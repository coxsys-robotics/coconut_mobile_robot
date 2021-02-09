#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

class set_initialPose():
	def __init__(self):
		self.amclPose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
		self.listener()
		
	def callback(self, app_pose_data):
		rospy.sleep(1)
		start_pos = PoseWithCovarianceStamped()
		start_pos.header.frame_id = "map"
		start_pos.header.stamp = rospy.Time.now()
		start_pos.pose.pose = app_pose_data
		self.amclPose_publisher.publish(start_pos)

	def listener(self):
		rospy.Subscriber("app_initial_pose", Pose, self.callback)
		rospy.spin()    
	
if __name__=="__main__":
	rospy.init_node("initial_pose_node", anonymous=True)
	process = set_initialPose()