#!/usr/bin/env python3
import rospy
from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList

if __name__ == "__main__":
    rospy.init_node("landmark_broadcaster")
    rospy.loginfo("initial node landmark_broadcaster")
    landmark_list = LandmarkList()
    landmark_pub = rospy.Publisher("/landmark", LandmarkList, queue_size=30)

    # Create home landmark
    home = LandmarkEntry()
    home.id = "home"
    home.tracking_from_landmark_transform.orientation.w = 1.0
    home.translation_weight = 1e7
    home.rotation_weight = 1e7

    # Add to list
    landmark_list.landmark.append(home)
    landmark_list.header.frame_id = "map"
    start_flag = True
    cnt = 0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if cnt > 60 and start_flag:
            start_flag = False
            landmark_list.landmark = []
            rospy.loginfo("home landmark already finish")
        landmark_list.header.stamp = rospy.Time.now()
        landmark_pub.publish(landmark_list)
        cnt += 1
        rate.sleep()
