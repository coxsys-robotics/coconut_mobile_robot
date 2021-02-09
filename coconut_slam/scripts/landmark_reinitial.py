#!/usr/bin/env python3
import rospy
from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList
from geometry_msgs.msg import PoseWithCovarianceStamped


initial_landmark = LandmarkEntry()
initial_landmark.id = "home"
initial_landmark.translation_weight = 5e7
initial_landmark.rotation_weight = 5e7

a = PoseWithCovarianceStamped()
a.pose.pose.position.x
a.pose.pose.orientation.x


initial_flag = False


def i_callback(data):
    global initial_landmark, initial_flag
    # initial_landmark.tracking_from_landmark_transform.orientation.x = data.pose.pose.orientation.x
    # initial_landmark.tracking_from_landmark_transform.orientation.y = data.pose.pose.orientation.y
    # initial_landmark.tracking_from_landmark_transform.orientation.z = data.pose.pose.orientation.z
    # initial_landmark.tracking_from_landmark_transform.orientation.w = data.pose.pose.orientation.w
    # initial_landmark.tracking_from_landmark_transform.position.x = data.pose.pose.position.y
    # initial_landmark.tracking_from_landmark_transform.position.y = -data.pose.pose.position.x
    initial_landmark.tracking_from_landmark_transform.orientation.x = 0
    initial_landmark.tracking_from_landmark_transform.orientation.y = 0
    initial_landmark.tracking_from_landmark_transform.orientation.z = 0
    initial_landmark.tracking_from_landmark_transform.orientation.w = 1
    initial_landmark.tracking_from_landmark_transform.position.x = 0.05
    initial_landmark.tracking_from_landmark_transform.position.y = 0
    initial_flag = True
    rospy.loginfo("Re-initial finish")


if __name__ == "__main__":
    rospy.init_node("landmark_broadcaster")
    rospy.loginfo("initial node landmark_broadcaster")
    landmark_list = LandmarkList()
    landmark_pub = rospy.Publisher("/landmark", LandmarkList, queue_size=30)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, i_callback)

    # Create home landmark

    # Add to list
    landmark_list.header.frame_id = "map"
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if initial_flag:
            initial_flag = False
            landmark_list.landmark = [initial_landmark]
        else:
            landmark_list.landmark = []
        landmark_list.header.stamp = rospy.Time.now()
        landmark_pub.publish(landmark_list)
        rate.sleep()
