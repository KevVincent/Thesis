#!/usr/bin/env python
import rospy
import cv2
import keras
import numpy as np 
import matplotlib.pyplot as plt 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError	
import segmentation_models.segmentation_models as sm 

class Prediction(object):

	def __init__(self, weights_path, backbone = 'inceptionresnetv2',  encoder_freeze=False, n_classes=4):
		self.backbone = backbone
		self.n_classes = n_classes # case for binary and multiclass segmentation
		self.classes = ['Background', 'Lane', 'Segmented_lane', 'Zebra'] if self.n_classes == 4 else ['Background', 'Lane']
		self.preprocess_fn = sm.get_preprocessing(self.backbone )
		self.activation = 'sigmoid' if self.n_classes == 1 else 'softmax'
		self.model = sm.Unet(self.backbone, classes=self.n_classes, activation=self.activation, encoder_freeze = encoder_freeze)
		self.optim = keras.optimizers.Adam(0.001)
		self.dice_loss = sm.losses.DiceLoss() 
		self.focal_loss = sm.losses.BinaryFocalLoss() if self.n_classes == 1 else sm.losses.CategoricalFocalLoss()
		self.total_loss = self.dice_loss + (1 * self.focal_loss)
		self.metrics = [sm.metrics.IOUScore(threshold=0.5), sm.metrics.FScore(threshold=0.5)]
		self.model.compile(self.optim, self.total_loss, self.metrics)
   		self.model.load_weights(weights_path)
		#topics to receive and send images
	def preprocessing(self, frame):
		return self.preprocess_fn(frame)

	def prediction(self, image, required_output = "Lane"):
		self.class_dict = {"Background":0, "Lane":1, "Segmented_lane":2, "Zebra":3}
		self.lane_type = self.class_dict[required_output]
		self.image = self.preprocessing(image)
		self.image = np.expand_dims(self.image, axis=0)
		self.predicted_mask = self.model.predict(self.image)
		self.mask = self.predicted_mask[..., self.lane_type].squeeze()
		self.mask[self.mask == self.lane_type]=255
		return self.mask

if __name__=='__main__':
	rospy.init_node('mask_predicter')
	predict = Prediction()
	rospy.spin()








