#!/usr/bin/env python

# Import necessary package
import rospy
import roslaunch
from os.path import expanduser

home = expanduser("~")

rospy.init_node('call_save_map_node', anonymous=False)

map_name = "test"
map_path = "{}/coconut_ws/src/coconut_uvc_bringup/map/".format(home)
map_filename = "{}".format(map_name)
map_file = map_path + map_filename

saveMap_node = roslaunch.core.Node(package='map_server', 
                                    node_type='map_saver', 
                                    name='map_saver',
                                    output="screen")
saveMap_node.args = "_f:={}".format(map_file)
saveMap_launch = roslaunch.scriptapi.ROSLaunch()
saveMap_launch.start()
saveMap_process = saveMap_launch.launch(saveMap_node)
while saveMap_process.is_alive():
    if saveMap_process.is_alive() == False:
        break
saveMap_process.stop()