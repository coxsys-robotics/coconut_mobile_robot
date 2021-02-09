#!/usr/bin/env python3

import rospy
import tf_conversions

import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

"""
Subscribe to odom topic thehn broadcast that odom frame under map frame
"""

class broadcaster:
    def __init__(self, map_frame='map', odom_topic='/odom'):
        self.odom_topic = odom_topic
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()

        self.tf_msg.header.frame_id = map_frame

        self.odom_sub = None

    def odom_callback(self, data):
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.child_frame_id = data.header.frame_id
        
        self.tf_msg.transform.translation.x = data.pose.pose.position.x
        self.tf_msg.transform.translation.y = data.pose.pose.position.y
        self.tf_msg.transform.translation.z = data.pose.pose.position.z
        
        self.tf_msg.transform.rotation.x = data.pose.pose.orientation.x
        self.tf_msg.transform.rotation.y = data.pose.pose.orientation.y
        self.tf_msg.transform.rotation.z = data.pose.pose.orientation.z
        self.tf_msg.transform.rotation.w = data.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(self.tf_msg)

    def start(self):
        rospy.init_node('map_odom_broadcaster')
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)


if __name__ == '__main__':
    
    bc = broadcaster(odom_topic='/t265/odom/sample')

    bc.start()

    rospy.spin()