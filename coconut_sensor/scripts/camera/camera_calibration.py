#!/usr/bin/env python

import os
import cv2
import glob
import numpy as np

# Initialize home directory
home = os.path.expanduser("~")

class Calibration:
    def __init__(self):
        self.gray = None

        # Variable
        self.mtx = None
        self.dist = None
        self.rvecs = None
        self.tvecs = None

        self.error = 0

        # Define the chess board rows and columns
        self.rows = 6
        self.cols = 9

        # Set the termination criteria for the corner sub-pixel algorithm
        self.criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 30, 0.001)

        # Prepare the object points: (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0). They are the same for all images
        self.objectPoints = np.zeros((self.rows * self.cols, 3), np.float32)
        self.objectPoints[:, :2] = np.mgrid[0:self.rows, 0:self.cols].T.reshape(-1, 2)

        # Create the arrays to store the object points and the image points
        self.objectPointsArray = []
        self.imgPointsArray = []
        self.calibrate_image()
        self.compute_error()
        self.show_result()

    def calibrate_image(self):
        
        # Loop over the image files
        for path in glob.glob('{}/coconut_ws/src/coconut_sensor/scripts/camera/pic/Chessboard'
                              '/*.jpg'.format(home)):
            # Load the image and convert it to gray scale
            image = cv2.imread(path)
            self.gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(self.gray, (self.rows, self.cols), None)

            # Make sure the chess board pattern was found in the image
            if ret:
                # Refine the corner position
                corners = cv2.cornerSubPix(self.gray, corners, (11, 11), (-1, -1), self.criteria)

                # Add the object points and the image points to the arrays
                self.objectPointsArray.append(self.objectPoints)
                self.imgPointsArray.append(corners)

                # Draw the corners on the image
                cv2.drawChessboardCorners(image, (self.rows, self.cols), corners, ret)

            # Display the image
            cv2.imshow('chess board', image)
            cv2.waitKey(500)

        # Calibrate the camera and save the results
        ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(self.objectPointsArray,
                                                                               self.imgPointsArray,
                                                                               self.gray.shape[::-1], None, None)
        np.savez('{}/coconut_ws/src/coconut_sensor/scripts/camera/variable/'.format(home),
                 "CalibrateVar.npz", mtx=self.mtx, dist=self.dist, rvecs=self.rvecs, tvecs=self.tvecs)

    def compute_error(self):
        # Print the camera calibration error
        for i in range(len(self.objectPointsArray)):
            imgPoints, _ = cv2.projectPoints(self.objectPointsArray[i], self.rvecs[i], self.tvecs[i],
                                             self.mtx, self.dist)
            self.error += cv2.norm(self.imgPointsArray[i], imgPoints, cv2.NORM_L2) / len(imgPoints)
        print("Total error: ", self.error / len(self.objectPointsArray))

    def show_result(self):
        # Load one of the test images
        img = cv2.imread('{}/coconut_ws/src/coconut_sensor/scripts/camera/pic/'
                         'Chessboard/Cap.jpg')
        h, w = img.shape[:2]
        # Obtain the new camera matrix and undistort the image
        newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        undistortedImg = cv2.undistort(img, self.mtx, self.dist, None, newCameraMtx)

        # Crop the undistorted image
        # x, y, w, h = roi
        # undistortedImg = undistortedImg[y:y + h, x:x + w]

        cv2.imshow('chess board', np.hstack((img, undistortedImg)))
        print("Original Image shape:", img.shape[:2])
        print("Calibrated Image shape:", undistortedImg.shape[:2])
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    process = Calibration()
