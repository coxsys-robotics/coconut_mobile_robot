#!/usr/bin/env python3

# Import necessary package
import rospy
import tf
from tf.transformations import euler_from_quaternion
import math

# Convert radian to degree format 
def rad2deg(radians):
    pi = math.pi
    degrees = (180 * radians) / pi
    return degrees

# Compute heading degree
def compute_headingDegree(pose_x, pose_y): 
    heading_yaw = math.atan2(pose_y, pose_x)
    heading_degree = rad2deg(heading_yaw)
    return heading_degree

# Main function
def find_nextAngle(target_frame):
    # rospy.init_node('findDiff_angle_node')
    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('base_footprint', target_frame , rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        break
    heading_degree = compute_headingDegree(trans[0], trans[1])
    
    return heading_degree

# heading_degree = find_diffAngle("charging_station")
# print(heading_degree)