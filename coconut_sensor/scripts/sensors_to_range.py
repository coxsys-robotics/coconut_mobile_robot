#!/usr/bin/env python3

import rospy
from std_msgs.msg import ByteMultiArray, UInt16MultiArray
from sensor_msgs.msg import Range

"""
subscribe to ultrasonic and bumper and publish 
range massage to use with obstacle avoidance
"""

class ultrasonic_to_range():
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
        self.rng_msg.range = range_data/100.0  ## cm -> m
        self.rng_msg.header.stamp = rospy.Time.now()
        self.rng_pub.publish(self.rng_msg)

def ultrasonic_callback(data):
    for index, value in enumerate(data.data):
        ultrasonics_list[index].publish(value)

if(__name__=="__main__"):
    ultrasonics_num = 10
    ultrasonics_list = []
    for i in range(ultrasonics_num):
        ultrasonics_list.append(ultrasonic_to_range(id=i))

    rospy.init_node("sensors_to_obstacle")
    rospy.Subscriber("ultrasonic_data",UInt16MultiArray, ultrasonic_callback)
    rospy.spin()


#### test version
# rng_msg = Range()
# rng_msg.header.frame_id = "ultrasonic_0"
# rng_msg.min_range = 0.2
# rng_msg.max_range = 7.2
# rng_msg.radiation_type = Range.ULTRASOUND
# rng_msg.field_of_view = 0.2
# rng_pub = rospy.Publisher("/ultrasonic_0", Range, queue_size=1)

# def callback(data):
#     global rng_msg
#     global rng_pub
#     rng_msg.range = data.data[0] /100.0
#     rng_msg.header.stamp = rospy.Time.now()

#     rng_pub.publish(rng_msg)


# if(__name__=="__main__"):
#     rospy.init_node("sensors_to_obstacle")
    # rospy.Subscriber("ultrasonic_data",UInt16MultiArray, callback)
    # rospy.spin()