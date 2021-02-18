#!/usr/bin/env python3

import rospy
import roslaunch
from os.path import expanduser
from std_msgs.msg import String

home = expanduser("~")

# lua_name = 'fortuna_1st_floor'
# map_name = 'fortuna_1st_floor'

def bringup_start():
	global parent		
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	cli_args = ['{}/coconut_ws/src/coconut_bringup/launch/coconut_bringup.launch'.format(home)]
	roslaunch_args = cli_args[1:]
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
	parent.start()

def bringup_shutdown():
	parent.shutdown()


class call_bringup_subscriber(object):

	def __init__(self):
		rospy.init_node('call_bringup_node', anonymous=True)
		self.call_bringup_command = None

	def callback(self,data):
		self.call_bringup_command = data.data

	def listener(self):
		rospy.Subscriber('call_launch', String, self.callback)


if __name__=="__main__":
	launchSub_node = call_bringup_subscriber()
	launchSub_node.listener()
	bringup_started = False
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = launchSub_node.call_bringup_command
			if command == "bringup" and not bringup_started:
				bringup_start()
				bringup_started = True
			elif command == "bringup_shutdown" and bringup_started:
				bringup_shutdown()
				bringup_started = False
			r.sleep()
	except Exception as e:
		print(e)