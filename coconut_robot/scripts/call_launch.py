#!/usr/bin/env python3 

import roslaunch
import rospy
from os.path import expanduser

home = expanduser("~")

rospy.init_node('call_launch_node', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ['{}/coconut_ws/src/coconut_uvc_bringup/launch/coconut_uvc_bringup.launch'.format(home)])
launch.start()
rospy.loginfo("started")
rospy.spin()