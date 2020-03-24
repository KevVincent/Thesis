import cv2
import time
import glob
import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.image as mpimg
from sklearn.cluster import DBSCAN

checkboard = (6,9)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
images = glob.glob('/home/robotics/Lane Detection/Image undistortion/calibration_images/image*.jpg')#path to chessboard images



def calibrate(images):
	# print('Calibrating camera')
	objpoints = []
	imgpoints = []
	objp = np.zeros((1, checkboard[0]*checkboard[1], 3), np.float32)
	objp[0,:,:2] = np.mgrid[0:checkboard[0], 0:checkboard[1]].T.reshape(-1, 2)
	for fnames in images:
		img = cv2.imread(fnames)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		ret,corners = cv2.findChessboardCorners(gray, (6,9), cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
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
	flag = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
	ret, K, D,rvecs, tvecs = cv2.fisheye.calibrate(objpoints, imgpoints, gray.shape[::-1],K, D, rvecs, tvecs, flag ,(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
	print('Calibrated-uundistorting')
	return K, D 

def undistort(image, K, D):
	h, w = image.shape[:2]
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w,h), cv2.CV_16SC2)
	undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	# img_dim = image.shape[:2][::-1]  
	# DIM = (480,640,3)  # dimension of the images used for calibration
	# scaled_K = K * img_dim[0] / DIM[0]  
	# scaled_K[2][2] = 1.0  
	# new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, img_dim, np.eye(3), balance=0)
	# map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, img_dim, cv2.CV_16SC2)
	# undist_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	return undistorted_img#, new_K
K, D = calibrate(images)
frames = sorted(glob.glob('/home/robotics/my/right/right_h2/frame*.jpg'))#path to distorted images
for image in frames:
	image = cv2.imread(image)
	undistorted = undistort(image, K,D)
