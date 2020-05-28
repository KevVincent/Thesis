#!/usr/bin/env python
#refer to https://github.com/qubvel/segmentation_models.git for the training algorithms
import rospy
import cv2
import argparse
import numpy as np 
import matplotlib.pyplot as plt 
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError	
import image_undistortion as undistort
import Kalman as K

from msg import worldpoints
import prediction as pred 
import segmentation_models.segmentation_models as sm 
"""
The following code snippet provides the user with the flexibility of specifying parameters like path to weights,
status of the encoder, and if Kalman filter should be used for lane prediction
"""
parser = argparse.ArgumentParser(description='This script converts the input camera frames to lane masks')
parser.add_argument('--backbone', type=str, default='inceptionresnetv2',
                    help='Name of the backbone used for U-net')
parser.add_argument('--encoder_frozen', type=bool, default=False,
                    help='Encoder frozen if True otherwise defrozen')
parser.add_argument('--weight_path', type=str,
                    help='Path to the weights')
parser.add_argument('--Kalman_filter', type=str, default='OFF',
                    help='ON if Kalman filter to used')
"""
This progran activates the camera and outputs the resulting mask predictions
"""

class LaneDetector(object):
	"""
	This class outputs the predicted mask and publishes the masks to the topic '/mask_topic'
	"""
	def __init__(self):
		"""
		initializes the variables and objects associated with with the class

		:type backbone: string
		:param backbone: name of the network to be used as the encoder for Unet

		:type encoder_frozen: boolean
		:param backbone: True if encoder was not retrained. False if encoder layers were trained

		:type weight_path: string 
		:param backbone: path to the weights file HDF5

		:type Kalman_filter: string
		:param backbone: ON if kalman filter to be used else OFF

		:param bridge: object of CvBridge() class

		:type kalman_flag: boolean
		:param backbone: initial value False. After the first dectection of lane turned to True.

		"""

		args = parser.parse_args()
		self.backbone = args.backbone
		self.encoder_frozen = args.encoder_frozen
		self.weight_path = args.weight_path
		self.Kalman_filter = args.Kalman_filter
		print('Loading the network....')
		self.model = pred(self.weight_path, self.backbone, self.encoder_frozen)
		print('Model ready')
		self.bridge = CvBridge()

		self.mask_pub = rospy.Publisher("mask_topic", Image)
		self.image_sub = rospy.Subscriber("image_topic",Image,self.frame2mask)
		self.kalman_flag = False

	def frame2mask(self, data):
		"""
		This function captures frames from camera and feeds to model(code-prediction.py)
		"""

	    try:
	    	print('Receiving frames')
	    	self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError as e:
	    	print('Error in receiving images')
	    	print(e)
		self.K, self.D = undistort.calibrate()#camera matrix and distortion matrix of the camera used for undistortion of frames
		self.frame = undistort.undistort(self.frame, self.K, self.D)#frames undistorting
		self.mask = self.model.prediction(self.frame)
		self.nonzero = np.nonzero(self.mask)#stores nonzeros indices
		if ( len(nonzero[0])!=0) & (len(nonzero[1])!=0 ):
	    """
	    once lane is predicted in the running session the kalman flag is set True.
	    if kalman is called without any lane pixels to start with 'ValueError' is raised
	    """
			self.kalman_flag = True
		if ( self.kalman_flag == True && self.Kalman_filter == True ):
			self.mask = K.converter(self.mask)
		#publishing the resuling mask to the topic
		print('Publishing mask to topic /mask_topic')
		self.mask_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
	def image2mask(self):
		"""
		This function takes in image from directory in repo and feeds to model(code-prediction.py)
		"""
		self.image = cv2.imread('./Test image/image.png')
		self.mask = self.model.prediction(self.image)
		cv2.imshow('predicted mask', self.mask)
		cv2.waitKey(0)


if __name__ == '__main__':
	rospy.init_node('LaneDetector')
	lane = LaneDetector()
	#lane.frame2mask()
	lane.image2mask()
	rospy.spin()

