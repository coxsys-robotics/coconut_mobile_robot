#!/usr/bin/env python3

import cv2
import yaml
import numpy as np

class undistort_image():
    def __init__(self,w=640,h=480,cal_mat_path="calibration.yml"):
        self.camera_matrix_path = cal_mat_path
        with open(self.camera_matrix_path, 'r') as stream:
            data = yaml.load(stream)

        self.mtx = data["camera matrix"]
        self.dist = data["distortion"]
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),1,(w,h))

        self.x,self.y,self.w,self.h = self.roi
        self.image = None
        
    def undistort(self,img):
        self.image = cv2.undistort(img, self.mtx, self.dist, None, self.newcameramtx)
        self.image = self.image[self.y:self.y+self.h, self.x:self.x+self.w]


if __name__=="__main__":
    mat_path = "calibration.yml"
    cap = cv2.VideoCapture(0)

    _,frame = cap.read()
    h, w = frame.shape[:2]

    undistort = undistort_image(w,h,mat_path)

    while(True):
        _,frame = cap.read()
        undistort.undistort(frame)

        cv2.imshow('distort',frame)
        cv2.imshow('undistort',undistort.image)
        # cv2.imshow('distorted vs undistorted', np.concatenate((frame,dst),axis=0))

        k = cv2.waitKey(1)
        if(k==ord('q')):
            break

    cv2.destroyAllWindows()