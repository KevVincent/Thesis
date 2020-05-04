#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  5 16:12:23 2020

@author: robotics
"""
#code to perform camera callibration using an image with four points under consideration
#camera base height = 9 cm.
import cv2
import numpy as np
import image_undistortion as cm

camera_matrix, dist_coef = cm.calibrate()

def CameraCalibration(mtx, dist):
    #This function uses camera matrix, distortion matrix, pixel coordinates,and real-world coordinates to
    #calculate 2D to 3D conversion matrix.
    #takes in camera matrix and distortion matrix
    #image points(2D) and object point(3D) with respect to image taken at the camera height mentioned above. 
	imgpoint =[[337.238, 396.456],[337.238,385.179],[276.467,383.3],[269.575,393.95]]#pixel coordinates of four point under considration
	imgpoint=np.array(imgpoint, np.float32)
	objpoint=[[0,24,0],[0,26.5,0],[7,26.5,0],[7,24,0]]#real world coordinates of the image under consideration with centre of camera as origin
	objpoint = np.array(objpoint, np.float32)
	rtvel, rvecs, tvecs = cv2.solvePnP(objpoint, imgpoint, mtx, dist)
	rmtx = cv2.Rodrigues(rvecs)
	tvecs = np.array(tvecs, np.float32)
	rmtx = rmtx[0]
    #'extrinsic matrix'
	ext_mtx = np.column_stack((rmtx, tvecs))
    #'projection matrix'
	proj_mtx = np.matmul(mtx, ext_mtx)
    #'homography matrix'
	hom_mtx = np.delete(proj_mtx, 2, 1)
	inverse = np.linalg.inv(hom_mtx)
	return inverse

def TwoD2ThreeD(x,y, inverse):
    #takes in x-pixel coordinate and y-pixel coordinate and returns the 3D position.
	point = np.array([x,y,1])
	three = np.matmul(inverse, point)
	x = three[0]/three[2]
	y = three[1]/three[2]
	return x, y

inverse = CameraCalibration(camera_matrix, dist_coef)
