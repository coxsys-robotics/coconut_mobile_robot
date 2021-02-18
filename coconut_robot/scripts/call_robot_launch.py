#!/usr/bin/env python3

import rospy
import roslaunch
from os.path import expanduser
from std_msgs.msg import String

home = expanduser("~")

# lua_name = 'fortuna_1st_floor'
# map_name = 'fortuna_1st_floor'

def robot_start():
	global parent		
	filename = rospy.get_param("/map_name", 'test')
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	cli_args = ['{}/coconut_ws/src/coconut_robot/launch/coconut_robot.launch'.format(home), "map_name:={}".format(filename), "lua_name:={}".format(filename)]
	roslaunch_args = cli_args[1:]
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
	parent.start()

def robot_shutdown():
	parent.shutdown()


class call_launch_subscriber(object):

	def __init__(self):
		rospy.init_node('call_robot_launch_node', anonymous=True)
		self.call_launch_command = None

	def callback(self,data):
		self.call_launch_command = data.data

	def listener(self):
		rospy.Subscriber('call_launch', String, self.callback)


if __name__=="__main__":
	launchSub_node = call_launch_subscriber()
	launchSub_node.listener()
	robot_started = False
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = launchSub_node.call_launch_command
			if command == "robot" and not robot_started:
				robot_start()
				robot_started = True
			elif command == "robot_shutdown" and robot_started:
				robot_shutdown()
				robot_started = False
			r.sleep()
	except Exception as e:
		print(e)