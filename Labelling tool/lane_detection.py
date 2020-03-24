import cv2
import time
import glob
import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.image as mpimg

checkboard = (6,9)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
images = glob.glob('/home/robotics/my/Thesis/images/120/image*.jpg')



def calibrate(images):
	print('Calibrating camera')
	objpoints = []
	imgpoints = []
	objp = np.zeros((1, checkboard[0]*checkboard[1], 3), np.float32)
	objp[0,:,:2] = np.mgrid[0:checkboard[0], 0:checkboard[1]].T.reshape(-1, 2)
	for fnames in images:
		img = cv2.imread(fnames)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		ret,corners = cv2.findChessboardCorners(gray, (6,9), None)
		if ret == True:
			image = cv2.drawChessboardCorners(img, (6,9),corners, ret)
			objpoints.append(objp)
			cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
			imgpoints.append(corners)
	N_OK = len(objpoints)
	objpoints = np.reshape(objpoints, (N_OK, 1, 6*9, 3))
	imgpoints = np.reshape(imgpoints, (N_OK, 1, 6*9, 2))
	rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
	tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
	K = np.zeros((3,3), np.float32)
	D = np.zeros((4,1), np.float32)
	flag = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
	ret, K, D,rvecs, tvecs = cv2.fisheye.calibrate(objpoints, imgpoints, gray.shape[::-1],K, D, rvecs, tvecs, flag ,(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
	print('Calibrated-uundistorting')
	return K, D 

def undistort(image, K, D):
	h, w = image.shape[:2]
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w,h), cv2.CV_16SC2)
	undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	return undistorted_img
	
def CameraCalibration(mtx, dist):
	imgpoint =[[324,418],[324,374],[213,374],[185.6,418.5]]

	# imgpoint=[[88.9, 395.705],[100.947, 53.03],[557.415,399.498],[549.828,50.508]]
	# #imgpoint=[[275, 204],[331, 204],[331, 308],[275, 308]]
	imgpoint=np.array(imgpoint, np.float32)
	# # print(imgpoint)
	objpoint=[[0, 20,0],[0,25,0],[4.5,25,0],[4.5,20,0]]
	# objpoint=[[0, 0,0],[11,0,0],[0,14.5,0],[0,11,14.5]]
	# #objpoint=[[0.0, 0.0, 0.0],[1.775, 0.0, 0.0],[1.775, 4.620, 0.0],[0.0, 4.620, 0.0]]
	objpoint = np.array(objpoint, np.float32)
	rtvel, rvecs, tvecs = cv2.solvePnP(objpoint, imgpoint, mtx, dist)
	rmtx = cv2.Rodrigues(rvecs)
	tvecs = np.array(tvecs, np.float32)
	rmtx = rmtx[0]
	ext_mtx = np.column_stack((rmtx, tvecs))
	proj_mtx = np.matmul(mtx, ext_mtx)
	hom_mtx = np.delete(proj_mtx, 2, 1)
	print('homography matrix')
	print(hom_mtx)
	inverse = np.linalg.inv(hom_mtx)
	return inverse

def TwoD2ThreeD(x,y, inverse):
	print('inverse homography matrix')
	print(inverse)
	point = np.array([x,y,1])
	three = np.matmul(inverse, point)
	print('x = ' + str(three[0]/three[2])+' cm')
	print('y = ' + str(three[1]/three[2])+' cm')

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
	return cv2.GaussianBlur(yellow_mask, (3,3),0)

def Threshold(image):
	binary = np.zeros_like(image)
	binary[(image>100) & (image<180)] = 1
	return binary

def RegionOfInterest(image):
	height, width = image.shape
	vertices = [(0,height), (0, int(height*0.75)), (width, int(height*0.75)),(width, height)]
	mask = np.zeros_like(image)
	color=255
	cv2.fillPoly(mask, np.array([vertices]) ,color)
	masked_image= cv2.bitwise_and(image, mask)
	return masked_image
def checkYellow(binary):
	nonzero = binary.nonzero()
	nonzeroy = np.array(nonzero[0])	
	nonzerox = np.array(nonzero[1])
	if len(nonzerox) == 0 or len(nonzeroy) == 0:
		return False
	else:
		return True
K, D = calibrate(images)

image = cv2.imread('/home/robotics/my/Thesis/codes/checker.jpg')#lane image to be converted to binary
undistorted = undistort(image, K,D)
hsv= Augmentation(undistorted)
yellow_mask = isolateYellow(hsv)
roi = RegionOfInterest(yellow_mask)
