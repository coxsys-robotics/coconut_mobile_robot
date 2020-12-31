#!/usr/bin/env python3

import math
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

"""
subscribe from t265 IMU acceleration. find amplitute and exclude gravity to find vibration.
Then publish calculated value.
"""
class accel_meter:
    def __init__(self):
        self.g = 9.8 ## gravity 
        self.constant = 0.18 ## constant value when robot is stationary
        self.freq = 1.0 / 62.7 ## unit: second

        rospy.init_node("acc_meter")
        self.acc_sub = rospy.Subscriber('/t265/accel/sample', Imu, self.acc_callback)
        self.acc_pub = rospy.Publisher('acc_no_g', Float32, queue_size=10)
        self.acc_msg = Float32()

        # self.vel_pub = rospy.Publisher('vel', Float32, queue_size=10)
        # self.vel_msg = Float32()

    def acc_callback(self, data):
        """
        Subscribed acc come in 3 dimension vector(x,y,z), so use Pythagorean addition to calculate amplitute
        and then exclude amplitute of gravity vector.  
        """
        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        az = data.linear_acceleration.z
        a = math.sqrt( pow(ax,2) +  pow(ay,2) +  pow(az,2) ) - self.g - self.constant
        self.acc_msg.data = a
        self.acc_pub.publish(self.acc_msg)

    def intergral_acc(self):
        """
        TODO: add some filter to make acc data more accurate
        """
        pass

if(__name__=="__main__"):
    acc = accel_meter()
    rospy.spin()