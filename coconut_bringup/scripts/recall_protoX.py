#!/usr/bin/env python3 

import sys
import time
import glob
import rospy
import rosnode
import roslaunch
from os.path import expanduser
from std_msgs.msg import String, UInt8

home = expanduser("~")
port = "/dev/ttyUSB0" #"/dev/nucleo"
node = "/protoX_node"
watch_time = None
timeout = 5
# baud = "115200"

"""
this function launch call_protoX.launch
"""
def protoX_start():
	global parent

	time.sleep(1)

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	cli_args = ['{}/coconut_ws/src/coconut_bringup/launch/call_protoX.launch'.format(home)]
	roslaunch_args = cli_args[1:]
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
	parent.start()
	
	time.sleep(2)
	# rospy.spin()


def protoX_shutdown():
	parent.shutdown()

def protoX_topic_callback(data):
	global watch_time
	watch_time = time.time()

###
# def check_node_list():
# 	interst_node = None
# 	node_list = rosnode.get_node_names()
	
# 	# print(node_list)

# 	if node in node_list:
# 		interst_node = True
# 	elif node not in node_list:
# 		interst_node = False
	
# 	return interst_node

"""
this function check usb port to see if specified port name exist or not.
"""
def check_usb_port():
	interst_usb_port = None

	if sys.platform.startswith('linux'):
		ports = glob.glob('/dev/[n]*')
	else:
		raise EnvironmentError('Unsupported platform')
	
	result = ports

	interst_usb_port = False
	if port in result:
		interst_usb_port = True

	return interst_usb_port

def main():
	global watch_time
	protoX_node_start = False
	r = rospy.Rate(10)
	while(not rospy.is_shutdown()):
		# interst_node = check_node_list()
		timeout_bool = False
		if(watch_time is None or time.time()-watch_time > timeout):
			timeout_bool = True
		interst_usb_port = check_usb_port()
		
		if (timeout_bool and interst_usb_port and not protoX_node_start):
			print("-----STARTING PROTOX-----")
			protoX_start()
		elif (not timeout_bool and interst_usb_port):
			protoX_node_start = True
		elif (timeout_bool and protoX_node_start):
			print("-----PROTOX TERMINATED-----")
			protoX_shutdown()
			protoX_node_start = False
		r.sleep()


if __name__=="__main__":
	rospy.init_node('protoX_watch_node', anonymous=True)
	# coconut_state_publisher = rospy.Publisher("coconut_state", String, queue_size =10)
	sub = rospy.Subscriber("powerboard_status", UInt8, protoX_topic_callback)
	try:
		main()
	except Exception as e:
		print(e)