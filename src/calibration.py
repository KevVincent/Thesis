#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  5 16:12:23 2020

@author: robotics
"""
#code to perform camera callibration using an image with four points under consideration
#camera base height = 9 cm.
import cv2
import rospy
import numpy as np
import image_undistortion as cm
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError	
from sensor_msgs.msg import Image
from LaneDetection.msg import worldpoints
	

class Calibration:
	def __init__(self):
	    self.bridge = CvBridge()
	    self.mask_sub = rospy.Subscriber("mask_topic",Image,self.Points)
	    self.point_pub = rospy.Publisher('3Dpoints', worldpoints)
	    self.world= worldpoints()
	    self.camera_matrix, self.dist_coef = cm.calibrate()


	def CameraCalibration(self, mtx, dist):
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

	def TwoD2ThreeD(self, x,y, inverse):
	    #takes in x-pixel coordinate and y-pixel coordinate and returns the 3D position.
		point = np.array([x,y,1])
		three = np.matmul(inverse, point)
		x = three[0]/three[2]
		y = three[1]/three[2]
		return x, y

	def Points(self, data):
	    try:
	      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError as e:
	      print(e)
		x_point = []
		y_point = []
		inverse = CameraCalibration(self.camera_matrix, self.dist_coef)
		nonzeros = np.nonzero(image)
		for i in range(len(nonzeros[0])):
			x, y = TwoD2ThreeD(nonzeros[1][i], nonzeros[1][i], inverse)
			x_point.append(x)
			y_point.append(y)
		self.world.x = x_point
		self.world.y = y_point
		self.point_pub.publish(self.world)


if __name__ == '__main__':
	rospy.init_node('transformation')
	i = Calibration()
	rospy.spin()
