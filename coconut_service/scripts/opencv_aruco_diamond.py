#!/usr/bin/env python3

"""
find aruco diamond tag using opencv and publish PoseStamp of tag with respect to camera
"""

import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

import numpy as np
from pyquaternion import Quaternion
from math import pi

import rospy
from geometry_msgs.msg import PoseStamped
from utils import rosImage2cv_bridge

from camera_undistort import undistort_image

from os.path import expanduser

home = expanduser("~")


"""
Use OpenCV and get pose of ArUco diamond marking
change pose.y to marker x axis in image range from 0 to 1
publish as PoseStamped
"""


# import argparse

# parser = argparse.ArgumentParser(description='Get image from ros topic and return pose of aruco tag in image.')
# parser.add_argument('--mode',type=str,help='single or diamond aruco tag.',default='diamond')
# parser.add_argument('--squarelength',type=float,help='length of black square in diamond mode.(cm)',default=0.055)
# parser.add_argument('--markerlength',type=float,help='length of each aruco tag.(cm)',default=0.033)
# parser.add_argument('--img_topic',type=str,help='topic name to subcribe ros image',default="/usb_cam/image_raw")


# args, unknown = parser.parse_known_args()

### which type of marker to use. "diamond" or "single"
mode = "diamond" #rospy.get_param("/camera_aruco/mode") #args.mode

### aruco diamond's parameters
squarelength = 3.816 #rospy.get_param("/camera_aruco/squarelength") #args.squarelength #centimeter
markerlength = 2.3 #rospy.get_param("/camera_aruco/markerlength") #args.markerlength #centimeter

### aruco single marker parameter
single_markerlength = 0.33 #rospy.get_param("/camera_aruco/markerlength") #args.markerlength

img_topic = "/camera/color/image_raw" #rospy.get_param("/camera_aruco/img_topic") #args.img_topic # "/test/camera1/image_raw"
use_ros_camera = False
camera_id = 6
fps = 30
w = 1280
h = 720

### path to camera matrix
mat_path =  "{}/coconut_ws/src/coconut_service/scripts/d415_720p.yml".format(home)
use_undistort = True

use_display = True

if __name__=="__main__":
    ### init ros 
    rospy.init_node("aruco_" + mode + "_pose")

    if(use_ros_camera):
        img_getter = rosImage2cv_bridge(img_topic, (h, w, 3) )
    else:
        vid = cv2.VideoCapture(camera_id)
        # vid.set(cv2.CAP_PROP_FORMAT, cv2.CAP_OPENCV_MJPEG)
        vid.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        vid.set(cv2.CAP_PROP_FRAME_HEIGHT , h)
        vid.set(cv2.CAP_PROP_FPS , fps)    


    ps = PoseStamped()
    
    aruco_pub = rospy.Publisher("/aruco_pose",PoseStamped,queue_size=10)
    aruco_msg = PoseStamped()

    # w,h = img_getter.color_image.shape[:2]
    camera_mat = np.asarray( [  [1.0, 0.0, 1.0],
                                [0.0, 1.0, 1.0],
                                [0.0, 0.0, 1.0]])
    if(use_undistort):
        undistort = undistort_image(w,h,mat_path)
        camera_mat = undistort.mtx
    

    ### opencv aruco detector
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    axisPoints = np.asarray(
        [[0,0,0],
        [0.250,0,0],
        [0,0.250,0],
        [0,0,0.250]]
    )

    ### in case camera_link in urdf not match camera frame in opencv
    quaternion_cameraLink_cameraFrame = Quaternion(1.0, 0.0, 0.0, 0.0) ## w,x,y,z format

    rospy.sleep(2)

    waitTime = 1

    while(not rospy.is_shutdown()):
        if(use_ros_camera):
            frame = img_getter.color_image.copy()
        else:
            _,frame = vid.read()

        if frame.shape[0]==0:
            continue
        
        if(use_undistort):
            undistort.undistort(frame)
            frame = undistort.image.copy()

        marker_coners, marker_ids, rejected_corners  = cv2.aruco.detectMarkers(frame,dictionary,parameters=parameters)
        # marker_coners, marker_ids, rejected_corners  = cv2.aruco.detectMarkers(undistort.image,dictionary,parameters=parameters)
        
        if marker_ids is not None:    

            if mode=="diamond":
                diamond_corners, diamond_ids = cv2.aruco.detectCharucoDiamond(frame,marker_coners,
                                                np.asarray(marker_ids),float(squarelength/markerlength))

                if diamond_ids is not None:
                    ### rvec : rotation vector
                    ### tvec : translation vector
                    rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(diamond_corners, squarelength, camera_mat, 0)

                    middle = [0,0]
                    for point in diamond_corners[0]:
                        middle[0] += point[0][0]
                        middle[1] += point[0][1]
                    middle[0] = int(middle[0] / 4)
                    middle[1] = int(middle[1] / 4)

                    # print(diamond_corners)
                    # print(l)
                    # print(tvecs)
                    # print('-----')
                    cv2.aruco.drawDetectedDiamonds(frame, diamond_corners, diamond_ids)
                    
                    for i in range(len(rvecs)):

                        rot_mat = cv2.Rodrigues(rvecs[i][0])[0]
                        # print(rot_mat) #checked

                        projected_point = cv2.projectPoints(axisPoints, rvecs[i], tvecs[i], camera_mat, 0)
                        # print(projected_point)

                        quaternion_cameraFrame_qr = Quaternion(matrix=rot_mat)
                        # print(quaternion_cameraFrame_qr)

                        quaternion_cameraLink_qr = quaternion_cameraLink_cameraFrame * quaternion_cameraFrame_qr
                        # print(quaternion_cameraLink_qr)

                        cv2.aruco.drawAxis(frame, camera_mat, 0, rvecs[i], tvecs[i], 5.25)
                        
                        aruco_msg.header.stamp = rospy.Time.now()
                        aruco_msg.header.frame_id = 'camera'
                        aruco_msg.pose.position.x = tvecs[i][0][0] * 0.01
                        aruco_msg.pose.position.y = middle[0] / w  #tvecs[i][0][1] * 0.01
                        aruco_msg.pose.position.z = tvecs[i][0][2] * 0.01
                        # aruco_msg.pose.position.x = tvecs[i][0][2]
                        # aruco_msg.pose.position.y = -tvecs[i][0][0]
                        # aruco_msg.pose.position.z = -tvecs[i][0][1]
                        aruco_msg.pose.orientation.w = quaternion_cameraLink_qr.w
                        aruco_msg.pose.orientation.x = quaternion_cameraLink_qr.x
                        aruco_msg.pose.orientation.y = quaternion_cameraLink_qr.y
                        aruco_msg.pose.orientation.z = quaternion_cameraLink_qr.z
                        aruco_pub.publish(aruco_msg)

            elif mode=="single":
                cv2.aruco.drawDetectedMarkers(frame,marker_coners,marker_ids)

                rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(marker_coners, single_markerlength, camera_mat, 0)

                for i in range(len(rvecs)):
                    projected_point = cv2.projectPoints(axisPoints, rvecs[i], tvecs[i], camera_mat, 0)
                    cv2.aruco.drawAxis(frame, camera_mat, 0, rvecs[i], tvecs[i], 0.25)
                    aruco_msg.header.stamp = rospy.Time.now()
                    aruco_msg.header.frame_id = 'camera'

                    rot_mat = cv2.Rodrigues(rvecs[i][0])[0]
                    quaternion_cameraFrame_qr = Quaternion(matrix=rot_mat)
                    quaternion_cameraLink_qr = quaternion_cameraLink_cameraFrame * quaternion_cameraFrame_qr
                    aruco_msg.pose.position.x = tvecs[i][0][0] * 0.01
                    aruco_msg.pose.position.y = tvecs[i][0][1] * 0.01
                    aruco_msg.pose.position.z = tvecs[i][0][2] * 0.01
                    # aruco_msg.pose.position.x = tvecs[i][0][2]
                    # aruco_msg.pose.position.y = -tvecs[i][0][0]
                    # aruco_msg.pose.position.z = -tvecs[i][0][1]
                    aruco_msg.pose.orientation.w = quaternion_cameraLink_qr.w
                    aruco_msg.pose.orientation.x = quaternion_cameraLink_qr.x
                    aruco_msg.pose.orientation.y = quaternion_cameraLink_qr.y
                    aruco_msg.pose.orientation.z = quaternion_cameraLink_qr.z
                    aruco_pub.publish(aruco_msg)

        if(use_display):
            cv2.namedWindow('aruco_diamond',0)
            cv2.imshow('aruco_diamond',frame)
            # cv2.imshow('aruco_diamond',undistort.image)

            k = cv2.waitKey(waitTime)
            if(k==ord('q')):
                break
            elif(k==ord('p')):
                waitTime = 1-waitTime

    cv2.destroyAllWindows()