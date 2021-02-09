#!/usr/bin/env python3 

import rospy
import rosbag
from os.path import expanduser
from std_msgs.msg import String

home = expanduser("~")

def filter_bag_start():		
	filename = rospy.get_param("/map_name", 'test')

	bag_path = "{}/coconut_ws/src/coconut_uvc_bag/bags/".format(home)
	bag_filename = "{}".format(filename)
	bag_file = bag_path + bag_filename
	
	rosbag.rosbag_main.filter_cmd(["{}.bag".format(bag_file),"{}_filtered.bag".format(bag_file),
								   "topic != '/tf' or topic == '/tf' and m.transforms[0].header.frame_id != 'map' and m.transforms[0].child_frame_id != 't265_odom_frame'"])
	coconut_state_publisher.publish("filter_bag_done")

class call_launch_subscriber(object):

	def __init__(self):
		
		self.call_launch_command = None

	def callback(self,data):
		self.call_launch_command = data.data

	def listener(self):
		rospy.Subscriber('call_launch', String, self.callback)


if __name__=="__main__":
	rospy.init_node('call_filter_bag_node', anonymous=True)
	coconut_state_publisher = rospy.Publisher("coconut_state", String, queue_size =10)

	launchSub_node = call_launch_subscriber()
	launchSub_node.listener()

	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = launchSub_node.call_launch_command
			if command == "filter_bag":
				filter_bag_start()
				launchSub_node.call_launch_command = None
			r.sleep()
	except Exception as e:
		print(e)