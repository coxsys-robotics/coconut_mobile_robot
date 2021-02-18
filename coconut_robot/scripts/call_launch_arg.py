#!/usr/bin/env python3 

import rospy
import roslaunch
from os.path import expanduser

rospy.init_node('call_launch_arg_node', anonymous=True)

home = expanduser("~")

bag_name = 'test'
lua_name = 'coxsys_slam'

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['{}/coconut_ws/src/coconut_bringup/launch/coconut_slam.launch'.format(home), "bag_name:={}".format(bag_name), "lua_name:={}".format(lua_name)]
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()
rospy.spin()