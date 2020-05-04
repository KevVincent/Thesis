import cv2
import numpy as np 
import math
"""
This code snippet does prediction on input mask using kalman filter.
The predicted parameters include mid point values and angle in which the lane is directed.
Two functions are used for kalman filter. One for midpoint prediction and other for angle prediction
for better results. This is due to unsatisfactory prediction using (6,3) Kalman filter
"""
length = 50
mid= [0,0]
angle = 0
"""
Kalman filter object creation for mid point(x, y) prediction.
"""
kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
predictedCoords = np.zeros((2, 1), np.float32)	
"""
Kalman filter object creation for angle prediction.
"""
kf2 = cv2.KalmanFilter(2, 1)
kf2.measurementMatrix = np.array([[1, 0]], np.float32)
kf2.transitionMatrix = np.array([[1, 1], [0, 1]],np.float32)
def mid_Estimate(coordX, coordY):
	"""
	This function corrects the measured mid point values
	"""
	measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
	kf.correct(measured)
	predicted = kf.predict()
	return predicted
	
def angle_estimate(angle):
	"""
	This function corrects the measured angle values
	"""
	measured = np.array([[np.float32(angle)]])
	kf2.correct(measured)
	predicted = kf2.predict()
	return predicted

def converter(image):
	global mid, angle
	image = np.dstack((image, image, image))
	nonzeros = []
	height, width = image.shape[:2]
	nonzero= np.nonzero(image)
	try:
		"""
		The code does he prediction while the there are lane pixels on the mask.
		As soon as there are no pixels a ValueError is raised.
		In this case previously predicted values are fed as inputs for prediction.
		"""
		for i in nonzero[0]:
			if i > int(height/2):
				nonzeros.append(i)
		y_positions = [min(nonzeros), max(nonzeros)]
		indices = [int(np.mean(np.where(nonzero[0]==y_positions[0]))),int(np.mean(np.where(nonzero[0]==y_positions[1])))]
		x_positions = [nonzero[1][indices[0]],nonzero[1][indices[1]]]
		mid = [(x_positions[0]+x_positions[1])/2, (y_positions[0]+y_positions[1])/2]
		radian = math.atan2((y_positions[1]-y_positions[0]), (x_positions[0]-x_positions[1]))
		calculated_angle = math.degrees(radian)
		predictedCoords = mid_Estimate(mid[0], mid[1] )
		angle = angle_estimate(calculated_angle)

	except(ValueError):
		predictedCoords = mid_Estimate(mid[0], mid[1] )
		angle = angle_estimate(calculated_angle)

	radian = math.radians(predictedCoords[1])
	print([math.cos(radian), math.sin(radian)])
	if predictedCoords[1] > 90:
		x2 = int(predictedCoords[0] + length*math.cos(radian))
		y2 = int(predictedCoords[1] - length*math.sin(radian))
	elif predictedCoords[1] <= 90:
		y2 = int(predictedCoords[1] - length*math.sin(radian))
		x2 = int(predictedCoords[0] + length*math.cos(radian))
	cv2.line(image, (mid[0],mid[1]), (x2,y2), (0,0,255), 9,30) 
	return image 


