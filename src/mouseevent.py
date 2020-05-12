import numpy as np 
import cv2
import glob
import matplotlib.pyplot as plt
mode = 0
pts=[]
color = (0,0,0)
imgs = glob.glob('image*.jpg')
circle_size = 10
def click_event(event, x, y, flags, param):
	global pts
	if mode==1:
		#for black pixels
		if event == cv2.EVENT_MOUSEMOVE:
			cv2.circle(c, (x,y), circle_size, (0,0,0), -1)
			cv2.circle(res, (x,y), circle_size, (0,0,0), -1)
	if mode == 2:
		#for white pixels
		if event == cv2.EVENT_MOUSEMOVE:
			cv2.circle(c, (x,y), circle_size, (255,255,255), -1)
			cv2.circle(res, (x,y), circle_size, (255,255,255), -1)
	if mode == 3:
		#to fill a polygon with black
		if event == cv2.EVENT_LBUTTONDOWN:
			#left click to chose the edges of required polygon
			pts.append((x,y))
			print(x)
			print(y)
		elif event == cv2.EVENT_RBUTTONDOWN:
			#right clich to fill the polygon
			pts = np.array(pts, dtype=np.int32)
			print(pts)
			cv2.fillPoly(c, [pts], (255,255,255))
			pts = []
	if mode == 4:
		if event == cv2.EVENT_LBUTTONDOWN:
			#same as mode 3. But for creating white polygon
			pts.append((x,y))
			print(x)
			print(y)
		elif event == cv2.EVENT_RBUTTONDOWN:
			pts = np.array(pts, dtype=np.int32)
			print(pts)
			cv2.fillPoly(c, [pts], (0,0,0))
			pts = []
		else:
			pass
	if mode == 5:
		if event == cv2.EVENT_LBUTTONDOWN:
			pts.append((x,y))
			print(x)
			print(y)
		elif event == cv2.EVENT_RBUTTONDOWN:
			pts = np.array(pts, dtype=np.int32)
			cv2.fillPoly(c, [pts], color)
			cv2.fillPoly(x, [pts], 255)
			print(pts)
			pts = []
		else:
			pass
	if mode == 6:
		if event == cv2.EVENT_LBUTTONDOWN:
			res[240:480,:]=(0,0,0)
			c[240:480,:]=(0,0,0)
		elif event == cv2.EVENT_RBUTTONDOWN:
			res[240:480,:]=(255,255,255)
			c[240:480,:]=(255,255,255)


undis = '/home/robotics/lane_detection/src/Test image/image.png'
image = cv2.imread(undis)
c = np.zeros_like(image)
x = cv2.imread(undis,0)
x=np.zeros_like(x)
# cv2.imshow('gdd',res)
# cv2.imshow('g',a)
#plt.show()
cv2.namedWindow('original')
cv2.setMouseCallback('original', click_event)

while(1):
	res = cv2.bitwise_and(image, image, mask=x)
	cv2.imshow('original',image)
	cv2.moveWindow('original',20,20)
	cv2.imshow('mask',c)
	cv2.moveWindow('mask',800,20)
	cv2.imshow('overlayed',res)
	cv2.moveWindow('overlayed',20,600)
	k = cv2.waitKey(1)
	if k == ord('s'):
#for saving the images.
		d=d+1
		mode=0
		break
	elif k==ord('d'):
		break
	elif k==ord('e'):
		color = (0,0,255)
	elif k==ord('r'):
		color = (0,255,255)
	elif k==ord('t'):
		color = (0,255,0)
	elif k==ord('<'):
		mode = 0
	elif k==ord('y') or k == ord('z'):
		mode = 1
	elif k == ord('x'):
		mode = 2
	elif k==ord('c'):
		mode = 3
	elif k==ord('v'):
		mode = 4
	elif k==ord('b'):
		mode = 5
	elif k==ord('n'):
		mode = 6
	elif k==ord('q'):
		circle_size+=2 if circle_size<25 else 0


	elif k==ord('a'):
		circle_size-=2 if circle_size>2 else 0
