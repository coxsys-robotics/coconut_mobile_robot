#!/usr/bin/env python3

# Import necessary package 
import rospy
import roslaunch
from read_actionFile import read_action
from read_roomFile import read_room

rospy.init_node('test_function_node')

room_data = read_room() 

robot_goal = room_data["1001"]

read_action([robot_goal[0], robot_goal[1], robot_goal[2], robot_goal[3]], "turn2currentRoom")
rospy.loginfo("Base footprint relative to map {} degree".format(read_action.Degree1))
rospy.loginfo("Target relative to map {} degree".format(read_action.Degree2))
rospy.loginfo("Robot start align with room by {} degree".format(read_action.rotate2currentRoom))

initialAlignment_node = roslaunch.core.Node(package='coconut_uvc_sensor', 
                                             node_type='rotateBy_odom.py', 
                                             name='rotateBy_odom_node')
initialAlignment_node.args = "_rotate_target:=%d" %(read_action.rotate2currentRoom)
initialAlignment_launch = roslaunch.scriptapi.ROSLaunch()
initialAlignment_launch.start()
initialAlignment_process = initialAlignment_launch.launch(initialAlignment_node)
while initialAlignment_process.is_alive():
    if initialAlignment_process.is_alive() == False:
        break
initialAlignment_process.stop()
