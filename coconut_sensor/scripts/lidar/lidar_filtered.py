#!/usr/bin/env python3

# Important Library

from sensor_msgs.msg import LaserScan
import rospy


class filterLidar(object):

    def __init__(self):
        self.data = None
        self.lidar_filtered_publisher = rospy.Publisher('/scan_filtered', LaserScan)
        self.listener()

    def callback(self,data):
        self.lidar_filtered_publisher.publish(data)

    def listener(self):
        rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=1000)
        rospy.spin()


if __name__=="__main__":
    rospy.init_node('lidar_filtered_node')
    filterLidar()
