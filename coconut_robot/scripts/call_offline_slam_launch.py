#!/usr/bin/env python3 

import rospy
import roslaunch
from os.path import expanduser
from std_msgs.msg import String

home = expanduser("~")

# bag_name = 'test'
# lua_name = 'test'

def offline_slam_start():
	global parent	
	filename = rospy.get_param("/map_name", 'test')	
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	cli_args = ['{}/coconut_ws/src/coconut_bringup/launch/offline_slam.launch'.format(home), "bag_name:={}".format(filename), "lua_name:={}".format(filename)]
	roslaunch_args = cli_args[1:]
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
	parent.start()

def offline_slam_shutdown():
	coconut_call_launch_publisher.publish("loop_save_map_shutdown")
	parent.shutdown()

class call_launch_subscriber(object):

	def __init__(self):
		rospy.init_node('call_offline_slam_launch_node', anonymous=True)
		self.call_launch_command = None

	def callback(self,data):
		self.call_launch_command = data.data

	def listener(self):
		rospy.Subscriber('call_launch', String, self.callback)


if __name__=="__main__":
	launchSub_node = call_launch_subscriber()
	coconut_call_launch_publisher = rospy.Publisher("call_launch", String, queue_size =10)
	launchSub_node.listener()
	slam_started = False
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = launchSub_node.call_launch_command
			if command == "offline_slam" and not slam_started:
				offline_slam_start()
				slam_started = True
			elif command == "offline_slam_shutdown" and slam_started:
				offline_slam_shutdown()
				slam_started = False
			r.sleep()
	except Exception as e:
		print(e)