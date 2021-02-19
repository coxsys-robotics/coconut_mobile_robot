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

# Compute turning angle to reduce rotation error
def compute_turningDegree(degree1, degree2): 
    turning_degree = degree2 - degree1
    return turning_degree

# Convert angle from quaternion to euler
def quar2euler(orientation_matrix):
    orientation_x = 0.0
    orientation_y = 0.0
    orientation_z = orientation_matrix[2]
    orientation_w = orientation_matrix[3]
    orientation_list = [orientation_x, orientation_y, orientation_z, orientation_w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

# Main function
def find_currentAngle(target_orientation):
    # rospy.init_node('findCurrent_angle_node')
    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', "base_footprint" , rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        break
    print(trans , rot)
    yaw1 = quar2euler(rot)
    yaw2 = quar2euler(target_orientation)
    # print("Yaw from map relative to base footprint:{}".format(yaw1))
    # print("Yaw from map relative to station2:{}".format(yaw2))
    degree1 = rad2deg(yaw1)
    degree2 = rad2deg(yaw2)
    # print("Degree from map relative to base footprint:{}".format(degree1))
    # print("Degree from map relative to station2:{}".format(degree2))
    turning_degree = compute_turningDegree(degree1, degree2) 
    
    return degree1, degree2, turning_degree

# turning_degree = find_CurrentAngle("charging_station")
# print(turning_degree)
