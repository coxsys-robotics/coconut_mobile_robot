#!/usr/bin/env python3

from roslaunch.parent import ROSLaunchParent
from battery_logger import battery_logger
from time import sleep
import rospy
import rosnode
import os
import sys

from std_msgs.msg import UInt8, UInt8MultiArray
# from geometry_msgs.msg import Twist

"""
***run this python file from startup with .sh via crontab
run roscore and any given .launch file
wait for ros shutdown message from ros topic
    -work flow-
shutdown message received --> kill all ros nodes --> shutdown bringup --> shutdown roscore -->
shutdown self node --> rospy.spin() will go off and then shutdown computer
"""

#TODO: ensure that ROS shutdown completely before shutdown computer

class core_bringup_manager():
    def __init__(self, path_to_bringup, shutdown_topic_name):
        ### get path to agv_v2_bringup.launch
        self.path_to_bringup = path_to_bringup

        ### get shutdown topic's name
        self.shutdown_topic_name = shutdown_topic_name
        self.shutdown_count = 0
        self.count_to_shutdown = 5

        ### node name for shutdown subscriber
        self.node_name = 'wait_for_shutdown'

        ### create ROSLaunchParent instance for roscore and agv_v2_bringup
        # self.roscore = ROSLaunchParent('core', [], is_core=True)
        self.bringup = ROSLaunchParent('bringup', [self.path_to_bringup], is_core=True)

        ### set this flag to True after self node killed
        self.shutdown_finished = False

    ### shutdown ros then shutdown computer
    def shutdown_all(self):
        self.shutdown_ros()
        self.shutdown_computer()

    ### kill all rosnodes except self --> shutdown bringup --> shutdown roscore --> kill self node
    def shutdown_ros(self):
        self.sub.unregister()
        neo = neopixel()
        neo.execute()
        print("---start killing nodes---")
        nodes_list = rosnode.get_node_names()
        nodes_list.remove('/' + self.node_name)
        rosnode.kill_nodes(nodes_list)
        print("---other nodes killed---")
        
        self.bringup.shutdown()
        print("---core shutdown---")
        rosnode.kill_nodes(['/' + self.node_name])
        print("---self node killed---")

        self.shutdown_finished = True
        # self.roscore.shutdown()
        

    ### function for shutdown computer
    def shutdown_computer(self):
        sudopass = "314159"
        command = "shutdown now"
        p = os.system('echo %s|sudo -S %s' % (sudopass, command))

    ### callback for shutdown message (twist: /debug_val.linear.y) 
    def shutdown_callback_twist(self, data):
        if(data.linear.y==0):
            self.shutdown_ros()

    ### callback for shutdown message (bool: /shutdown) 
    def shutdown_callback_bool(self, data):
        if(data.data):
            self.shutdown_ros()

    def shutdown_callback_uint8(self, data):
        if(data.data==4):
            self.shutdown_ros()
        elif(data.data==0):
            self.shutdown_count+=1
            if(self.shutdown_count >= self.count_to_shutdown):
                self.shutdown_ros()
        else:
            self.shutdown_count = 0

    ### start roscore and bringup and then start node subscribe for shutdown message 
    def start_ros(self):
        # self.roscore.start()
        self.bringup.start()

        sleep(5)
        self.sub = rospy.Subscriber(self.shutdown_topic_name, UInt8, self.shutdown_callback_uint8, queue_size=10)
        rospy.init_node(self.node_name)

class neopixel():
    def __init__(self):
        ## neopixel mode ##
        self.neopixel_mode_msg = UInt8()
        self.neopixel_mode_publisher = rospy.Publisher("neopixel_mode", UInt8, queue_size=10)
        ## neopixel rgb : [r0,g0,b0, r1,g1,b1, r2,g2,b2] ##
        self.neopixel_rgb_msg = UInt8MultiArray()
        self.neopixel_rgb_publisher = rospy.Publisher("neopixel_rgb", UInt8MultiArray, queue_size=10)
        rospy.init_node("last_neopixel")

    def execute(self):
        self.neopixel_rgb_msg.data = [10,10,30]
        self.neopixel_mode_msg.data = 8
        self.neopixel_rgb_publisher.publish(self.neopixel_rgb_msg)
        sleep(1)
        self.neopixel_mode_publisher.publish(self.neopixel_mode_msg)
        sleep(1)

if __name__=='__main__':
    sleep(5)
    manager = core_bringup_manager(sys.argv[1], sys.argv[2]) 
    manager.start_ros()
    # rospy.spin()
    sleep(5)
    Logger = battery_logger()
    rate = rospy.Rate(1)

    while not manager.shutdown_finished and not rospy.is_shutdown():
        Logger.rate.sleep()
        Logger.log_to_file()

    ### after shutdown self node, wait for 5 more seconds then shutdown computer
    sleep(5.0)
    manager.shutdown_computer()
