#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from cartographer_ros_msgs.msg import LandmarkList, LandmarkEntry

"""
lidar pepperl-fuchs axis
front is the side with button. back is the side which cable pluged in.
           180
        <- <- <-
    V     front     ^
270 V left up right ^   90
    V     back      ^
        -> -> ->
            0
"""
# import cv2
# import numpy as np

import math
import time
import random

class point_tracker():
    def __init__(self, point_id, position, intensity, distance, init_time, decay_time=2, max_delta=80, max_distance=0.2):
        """ param
        point_id (string): id of this point.
        position (int): position of this point. int because used with 1d data.
        intensity (float): intensity value of each point.
        distance (float): range data from lidar (metre).
        init_time (float): time that this point was found (time.time() in this case).
        decay_time (float): longest period to keep point when it is not found.
        max_delta (float): maximum cost to accept as same point (may need tuning for each cost function).
        max_distance (float): maximum distance tolerance for landmark between each iteration (range: (-max_distance, +max_distance)) (metre)).
        """
        self.id = point_id
        self.position = position
        self.intensity = intensity
        self.distance = distance
        self.latest_time = init_time
        self.max_delta = max_delta
        self.max_distance = max_distance
        self.life_time = decay_time
        self.color = (random.randrange(0,255),random.randrange(0,255),random.randrange(0,255))

        self.timed_out = False

    def is_same_point(self, pos_to_check, intensity, distance, pos_range):
        ### flow 
        # cost = somecalculation()
        # if(cost < self.max_delta):
        #   latest_time = current_time
        #   position = pos_to_check
        #   intensity = intensity
        #   return True
        # elif(current_time - latest_time > decay_time):
        #   set self timed out flag
        # else:
        #   return False
        ### ###
        cost = self.simple_cost(pos_to_check)
        if( (cost < self.max_delta or cost > (pos_range - self.max_delta) ) and abs(self.distance - distance) < self.max_distance):
            self.latest_time = time.time()
            self.timed_out = False
            self.position = pos_to_check
            self.intensity = intensity
            self.distance = distance
            return True
        else:
            # print(time.time() - self.latest_time)
            self.check_timed_out()
        return False
    
    def check_timed_out(self):
        if(time.time() - self.latest_time > self.life_time):
            self.timed_out = True

    def simple_cost(self, pos_to_check):
        return abs(self.position - pos_to_check)

class Intensity_to_landmarks():
    def __init__(self):
        self.landmark_topic = rospy.get_param("~landmark_topic", default="/landmark")
        self.lidar_topic = rospy.get_param("~lidar_topic", default="/scan")
        self.landmark_frame = rospy.get_param("~landmark_frame", default="lidar_link")

        ### point with less intensity than threshold will not be accepted as landmark
        self.intensity_threshold = rospy.get_param("~threshold", default=1400)

        ### amount of point each lidar rotation
        self.measurement_amount = 3600
        ### resolution for each point (degree)
        self.resolution = 0.1

        self.landmark_msg = LandmarkList()
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_callback)
        self.landmark_pub = rospy.Publisher(self.landmark_topic, LandmarkList, queue_size=10)
        
        self.landmark_list = []
        self.id_count = 0
        
        ### use to visualize. store min, max from one point ###
        # self.min = 9999
        # self.max = 0
        ### ### ###

        ### use to visualize with opencv ###
        # self.max_inten = 2000
        # self.canvas = np.zeros((self.max_inten, 3600, 3), dtype=np.uint8)
        ### ### ###

    def lidar_callback(self,data):
        ### Get position of local maxima(s) 
        maximas = self.local_maxima_for_dummy(data.intensities, self.intensity_threshold)
        
        ### if there are maxima(s)
        if(len(maximas) > 0):
            ### for each point in landmark_list, check if maxima match with point.
            for point in list(self.landmark_list):
                ### if all other maxima(s) are matched with other landmarks then remove this landmark.
                if(len(maximas)==0 ):
                    # print("disappeared")
                    point.check_timed_out()

                for maxima in list(maximas):
                    ### if match, landmark data will be updated and then remove maxima from list.
                    if(point.is_same_point(maxima, data.intensities[maxima], data.ranges[maxima], self.measurement_amount)):
                        maximas.remove(maxima)
                        break

                ### if a landmark is not found longer than "decay_time", it will be removed.
                if(point.timed_out):
                    print("time_out")
                    self.landmark_list.remove(point)
            ###if there are any maxima(s) left after matching, add them as new landmark.
            if(len(maximas) > 0):
                for i in maximas:
                    self.landmark_list.append(point_tracker(str(self.id_count), i, data.intensities[i], data.ranges[i], time.time()))
                    self.id_count += 1

        self.landmark_msg = LandmarkList()
        self.landmark_msg.header.stamp =  rospy.Time.now()
        self.landmark_msg.header.frame_id = self.landmark_frame
        ### for each landmark in landmark_list, add LandmarkEntry to cartographer LandmarkList message
        for landmark in self.landmark_list:
            tmp = LandmarkEntry()
            tmp.id = landmark.id
            angle = math.radians((self.resolution * landmark.position) - 180.0)
            tmp.tracking_from_landmark_transform.position.x = data.ranges[landmark.position] * math.cos(angle)
            tmp.tracking_from_landmark_transform.position.y = data.ranges[landmark.position] * math.sin(angle)
            tmp.tracking_from_landmark_transform.orientation.w = 1
            tmp.translation_weight = 1.0
            tmp.rotation_weight = 0
            self.landmark_msg.landmark.append(tmp)
        self.landmark_pub.publish(self.landmark_msg)


        # self.landmark_msg = LandmarkList()
        # self.landmark_msg.header.stamp = rospy.Time.now()
        # self.landmark_msg.header.frame_id = self.landmark_frame
        ### For each maxima, add landmark to catographer landmark list
        # for index,value in enumerate( maximas):
        #     tmp = LandmarkEntry()
        #     tmp.id = str(index)
        #     angle = math.radians((self.resolution * value) - 180.0)
        #     tmp.tracking_from_landmark_transform.position.x = data.ranges[value] * math.cos(angle)
        #     tmp.tracking_from_landmark_transform.position.y = data.ranges[value] * math.sin(angle)
        #     tmp.tracking_from_landmark_transform.position.z = 0
        #     # tmp.tracking_from_landmark_transform.orientation.x = 0
        #     # tmp.tracking_from_landmark_transform.orientation.y = 0
        #     # tmp.tracking_from_landmark_transform.orientation.z = 0
        #     tmp.tracking_from_landmark_transform.orientation.w = 1
        #     tmp.translation_weight = 1.0
        #     tmp.rotation_weight = 0

        #     self.landmark_msg.landmark.append(tmp)
        # self.landmark_pub.publish(self.landmark_msg)

        ### Plot intensity value with opencv ###
        # tmp = np.zeros((self.max_inten, 3600, 3), dtype=np.uint8)
        # for i, inten in enumerate( data.intensities):
        #     if(i>0):
        #         tmp = cv2.line(tmp, (i-1, self.max_inten - int(data.intensities[i-1]) - 1),
        #                      (i, self.max_inten - int(inten) - 1), (255,255,255), 2)
        #     tmp[self.max_inten - int(inten) - 1][i] = (255,255,255)
        # for point in self.landmark_list:
        #     tmp = cv2.line(tmp, (point.position, 70),
        #         (point.position, int(self.max_inten/2)), point.color, 2)
        #     tmp = cv2.putText(tmp, point.id, (point.position, 50), cv2.FONT_HERSHEY_SIMPLEX ,  
        #            1, point.color, 2) 
        # self.canvas = tmp.copy()
        ### ### ###
        
        ### Get min, max of one point. Use to see accuracy of measurement ###
        # if(data.ranges[3150] < self.min ):
        #     self.min = data.ranges[3150]
        # elif(data.ranges[3150] > self.max):
        #     self.max = data.ranges[3150]
        # print(self.min)
        # print(self.max)
        # print("-------")
        ### ### ###

    ### find position of local maxima(s) in 1D input data
    # METHOD 1 : find maxima(s) that has value above "threshold" and not close to other maxima(s) by "distance" unit
    def local_maxima_for_dummy(self, data_1D, threshold, distance=30):
        """
            If data goes above threshold, set flag up. When data goes below threshold, set flag down.
            Then find middle position between flag up and down then add it to maxima list.
            params:
                data_1D: list of data to find local maxima(s)
                threshold: value to determine whether data is considered as maxima or not
                distance: if maximas are close than this value then count as just one
            return:
                list of position of maxima(s).
        """
        maxima_list = []
        start_maxima = False
        start_position = 0
        end_position = 0
        for i, value in enumerate( data_1D):
            if (value > threshold and not start_maxima):
                start_maxima = True
                start_position = i
            elif(value < threshold and start_maxima):
                start_maxima = False
                end_position = i
                pos = int((start_position+end_position)/2)
                if(len(maxima_list)>0):
                    if( pos - maxima_list[-1] < distance):
                        maxima_list[-1] = int((pos + maxima_list[-1])/2)
                    else:
                        maxima_list.append(pos)        
                else:
                    maxima_list.append(pos)
        return maxima_list

    # def denoise(self, data_1D):
        

if(__name__=="__main__"):
    print("start")
    rospy.init_node("inten_to_landmarks")
    itl = Intensity_to_landmarks()
    rate = rospy.Rate(60)

    display = False
    # cv2.namedWindow("lidar", cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
        if(display):
            cv2.imshow("lidar", itl.canvas)
            # print(len(itl.landmark_list))
            key = cv2.waitKey(1)
            if (key==ord('q')):
                break
            elif (key==ord('i')):
                for lm in itl.landmark_list:
                    print(lm.id)
                print("------")
            elif(key==ord('c')):
                print(itl.id_count)
        # itl.landmark_pub.publish(itl.landmark_msg)
        rate.sleep()
