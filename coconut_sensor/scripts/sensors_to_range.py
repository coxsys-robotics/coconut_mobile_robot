#!/usr/bin/env python3

import rospy
from std_msgs.msg import ByteMultiArray, UInt16MultiArray
from sensor_msgs.msg import Range

"""
subscribe to ultrasonic and bumper and publish 
range massage to use with obstacle avoidance
"""

class ultrasonic_to_range():
    """
    base class for ultrasonic sensor.
    add value to range message's parameters.
    when ultrasonic message received, publish range message.
    """
    def __init__(self, id=0, min_range=0.2, max_range=7.2,fov=0.2):
        sensor_id = "ultrasonic_{0}".format(id)
        self.rng_msg = Range()
        self.rng_msg.header.frame_id = sensor_id
        self.rng_msg.min_range = min_range
        self.rng_msg.max_range = max_range
        self.rng_msg.radiation_type = Range.ULTRASOUND
        self.rng_msg.field_of_view = fov

        self.rng_pub = rospy.Publisher( sensor_id, Range, queue_size=1)

    def publish(self, range_data):
        self.rng_msg.range = range_data/100.0  ## convert from cm to m
        self.rng_msg.header.stamp = rospy.Time.now()
        self.rng_pub.publish(self.rng_msg)

def ultrasonic_callback(data):
    for index, value in enumerate(data.data):
        ultrasonics_list[index].publish(value)

if(__name__=="__main__"):
    ultrasonics_num = 10 ## how many ultrasonic sensors
    ultrasonics_list = [] ## list to store ultrasonic_to_range instance
    for i in range(ultrasonics_num):
        ultrasonics_list.append(ultrasonic_to_range(id=i))

    rospy.init_node("sensors_to_obstacle")
    rospy.Subscriber("ultrasonic_data",UInt16MultiArray, ultrasonic_callback)
    rospy.spin()
    