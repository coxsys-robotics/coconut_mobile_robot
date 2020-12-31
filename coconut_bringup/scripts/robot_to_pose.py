#!/usr/bin/env python 
import rospy
import tf

from geometry_msgs.msg import Pose

"""
read robot pose from tf and publish as Pose.
"""
class tf_to_pose():
    def __init__(self, map_frame, robot_frame):
        rospy.init_node("tf_to_pose")

        self.listener = tf.TransformListener()
        self.pose_msg = Pose()
        self.pose_pub = rospy.Publisher("pose_from_robot",Pose, queue_size=10)
        
        self.map_frame = map_frame
        self.robot_frame = robot_frame

    """
    robot move in 2d plain so use only position x,y and orientation.
    """
    def get_and_publish_pose(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.map_frame, self.robot_frame, rospy.Time(0))
            self.pose_msg.position.x = trans[0]
            self.pose_msg.position.y = trans[1]
            self.pose_msg.orientation.x = rot[0]
            self.pose_msg.orientation.y = rot[1]
            self.pose_msg.orientation.z = rot[2]
            self.pose_msg.orientation.w = rot[3]

            self.pose_pub.publish(self.pose_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

"""
tf from map_frame to robot_frame
"""
if __name__=="__main__":
    map_frame = '/map'
    robot_frame = '/base_footprint'
    pose_reader = tf_to_pose(map_frame, robot_frame)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pose_reader.get_and_publish_pose()
        rate.sleep()