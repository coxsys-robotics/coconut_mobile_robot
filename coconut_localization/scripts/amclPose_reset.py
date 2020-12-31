#!/usr/bin/env python

# Important Library

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy

import sys

import roslaunch

class set_amclPose():
    def __init__(self):
        self.amclPose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.fist_time = True
        
    def reset_amclPose(self, amcl_pose_data):

        rospy.sleep(1)

        start_pos = PoseWithCovarianceStamped()
        start_pos.header.frame_id = "map"
        
        if self.fist_time == True:
            self.old_position = amcl_pose_data
            self.fist_time = False

        x_changing = abs(self.old_position.pose.pose.position.x - amcl_pose_data.pose.pose.position.x)  
        y_changing = abs(self.old_position.pose.pose.position.y - amcl_pose_data.pose.pose.position.y)


        if x_changing > 2 or y_changing > 2:
            self.old_position.header.stamp = amcl_pose_data.header.stamp
            
            start_pos = self.old_position

            self.amclPose_publisher.publish(start_pos)
            
        else: 
            self.old_position = amcl_pose_data


class amclPose_subscriber(object):

    def __init__(self):
        self.pose = None

    def callback(self,data):
        self.pose = data

    def listener(self):
        rospy.init_node('amclPose_reset_node')
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.callback)


if __name__=="__main__":
    amclSub_node = amclPose_subscriber()
    amclSub_node.listener()

    rate = rospy.Rate(1)

    resetAmcl_node = set_amclPose()

    while(not rospy.is_shutdown()):
        amcl_pose_data = amclSub_node.pose
        if amcl_pose_data != None:
            resetAmcl_node.reset_amclPose(amcl_pose_data)
        rate.sleep()