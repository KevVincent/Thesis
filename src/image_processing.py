import cv2
import time
import glob
import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.image as mpimg
import image_undistortion as cm
rospy.init_node('image_undistortion')
def changeToHSV(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
def Augmentation(image):
	transformed = changeToHSV(image)
	H, S, V = cv2.split(transformed)
	kernal = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])
	hsv = cv2.filter2D(transformed, -1, kernal)
	hsv[:,:,0] = hsv[:,:,0] * 1.4
	hsv[:,:,1] = hsv[:,:,1] * 1.4
	hsv[:,:,2] = hsv[:,:,2] * 0.6
	return hsv
def isolateYellow(image):
	low_threshold = np.array([20, 100, 100], dtype=np.uint8)
	high_threshold = np.array([55, 255, 255], dtype=np.uint8)
	yellow_mask = cv2.inRange(image, low_threshold,high_threshold)
	return yellow_mask
def GaussianBlur(image):
	return cv2.GaussianBlur(image, (3,3),0)

def Threshold(image):
	binary = np.zeros_like(image)
	binary[(image>100) & (image<180)] = 1
	return binary

def RegionOfInterest(image):
	height, width = image.shape
	vertices = [(0,height), (0, int(height*0.5)), (width, int(height*0.5)),(width, height)]
	mask = np.zeros_like(image)
	color=255
	cv2.fillPoly(mask, np.array([vertices]) ,color)
	masked_image= cv2.bitwise_and(image, mask)
	return masked_image
def checkYellow(binary):
	nonzero = binary.nonzero()
	nonzeroy = np.array(nonzero[0])	
	nonzerox = np.array(nonzero[1])
	if len(nonzerox) <20 or len(nonzeroy) <20:
		return False
	else:
		return True

def getmedian(image):
	if checkYellow(image)==True:
		nonzero = image.nonzero()
		nonzeroy = np.array(nonzero[0])	
		nonzerox = np.array(nonzero[1])
		x = nonzerox[int(len(nonzerox)/2)]
		y = nonzeroy[int(len(nonzeroy)/2)]
		return x ,y
	elif checkYellow(image)==False:
		x = 0
		y = 0
		return x, y


K, D = cm.calibrate()
vid = cv2.VideoCapture('/home/robotics/my/Thesis/meeting/kalman.avi')
while(1):
	ret, image = vid.read()
	undistorted = undistort(image, K,D)
	hsv= Augmentation(undistorted)
	blurred = GaussianBlur(hsv)
	yellow_mask = isolateYellow(blurred)
	roi = RegionOfInterest(yellow_mask) 
	binary = np.zeros_like(roi)
	binary[roi!=0] = 1
	inverse_mtx = CameraCalibration(K, D)
	x, y = getmedian(roi)
	x, y= TwoD2ThreeD(x,y,inverse_mtx)
	print(x)
	print(y)
	cv2.imshow('dd', roi)
	cv2.waitKey(0)
