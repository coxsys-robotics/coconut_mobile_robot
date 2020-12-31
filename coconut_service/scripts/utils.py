#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image


class rosImage2cv_bridge:
    """
    Convert sensor_msgs/Image to cv2 image (numpy)
    Attributes:
        self.color_image    cv2 image

    Example:
        a = rosImage2cv_bridge("/camera/color/image")
        cv2.imshow('frame',a.color_image)

    """

    def __init__(self, topic="/camera/color/image",res=(0, 0, 3), dtype=np.uint8):
        self.res = res
        self.tmp_img = None
        self.color_image = np.zeros(self.res,dtype=dtype)
        self.sub_image = rospy.Subscriber(topic, Image, self.callback, queue_size=100)
        self.h = res[0]
        self.w = res[1]
        self.dtype = dtype

    def callback(self, data):
        if(self.h==0 or self.w==0):
            self.h = data.height
            self.w = data.width
        temp = np.frombuffer(data.data, dtype=self.dtype)
        self.tmp_img = temp.reshape((self.h, self.w, self.res[2]))
        if self.res[2]!=1:
            self.tmp_img = cv2.cvtColor(self.tmp_img, cv2.COLOR_RGB2BGR)
        self.color_image = self.tmp_img

if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("Starting RosNode.")
    a = rosImage2cv_bridge()
    while True:
        cv2.imshow('frame', a.color_image)
        k = cv2.waitKey(1)
        if k == ord('q'):
            cv2.destroyAllWindows()
            exit()
