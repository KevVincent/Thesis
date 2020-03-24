#algorithm to label images using open cv mouse events
import numpy as np 
import cv2
import glob
import matplotlib.pyplot as plt
mode = 0
pts=[]
imgs = glob.glob('image*.jpg')
circle_size = 10
def click_event(event, x, y, flags, param):
	global pts
	if mode==1:
		if event == cv2.EVENT_MOUSEMOVE:
			cv2.circle(c, (x,y), circle_size, (0,0,0), -1)
			cv2.circle(res, (x,y), circle_size, (0,0,0), -1)
	if mode == 2:
		if event == cv2.EVENT_MOUSEMOVE:
			cv2.circle(c, (x,y), circle_size, (255,255,255), -1)
			cv2.circle(res, (x,y), circle_size, (255,255,255), -1)
	if mode == 3:
		if event == cv2.EVENT_LBUTTONDOWN:
			pts.append((x,y))
			print(x)
			print(y)
		elif event == cv2.EVENT_RBUTTONDOWN:
			pts = np.array(pts, dtype=np.int32)
			print(pts)
			cv2.fillPoly(c, [pts], (255,255,255))
			pts = []
	if mode == 4:
		if event == cv2.EVENT_LBUTTONDOWN:
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
			height, width = c.shape
			print(c.shape)
			pts = np.array(pts, dtype=np.int32)
			x = np.zeros_like(c)
			cv2.fillPoly(x, [pts], 255)
			c[x==255]=255
			c[x!=255]=0
			print(x.shape)
			print(pts)
			
			print(pts)
			#cv2.fillPoly(c, [pts], (0,0,0))
			pts = []
		else:
			pass
	if mode == 6:
		if event == cv2.EVENT_LBUTTONDOWN:
			res[240:480,:]=(0,0,0)
			c[240:480,:]=(0)
		elif event == cv2.EVENT_RBUTTONDOWN:
			res[240:480,:]=(255,255,255)
			c[240:480,:]=(255)
	# if mode == 6:
		
	# 	if event == cv2.EVENT_LBUTTONDOWN:
	# 		pts.append((x,y))
	# 		print(x)
	# 		print(y)
	# 	elif event == cv2.EVENT_RBUTTONDOWN:
	# 		check = np.zeros_like(res)
	# 		pts = np.array(pts, dtype=np.int32)
	# 		print(pts)
	# 		cv2.fillPoly(check, [pts], 255,-1)
	# 		c[check==(0,0,0)]=(0)
			

# def Prev(b,d):
# 	a=imgs[b][5:]
# 	image = cv2.imread(imgs[b])
# 	distorted = '/home/robotics/my/right/right_h2/frame'+str(a)
# 	distorted = cv2.imread(distorted)
# 	b='/home/robotics/my/right/right_h2/Untitled Folder/image'+str(a)
# 	c = cv2.imread(b,0)
# 	print(b)
# 	res = cv2.bitwise_and(image, image, mask=c)
# 	# cv2.imshow('gdd',res)
# 	# cv2.imshow('g',a)
# 	#plt.show()
# 	cv2.namedWindow('dd')
# 	cv2.setMouseCallback('dd', click_event)

# 	while(1):
# 		res = cv2.bitwise_and(image, image, mask=c)
# 		cv2.imshow('dd',res)
# 		cv2.imshow('ff',c)
# 		cv2.imshow('f',image)
# 		k = cv2.waitKey(1)
# 		if k == ord('s'):
# 			# a=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY)
# 			# binary = np.zeros_like(a)
# 			# binary[img!=0]=1
# 			cv2.imwrite('/home/robotics/my/Thesis/Dataset/binary/gt'+str(d-1)+'.png', c)
# 			cv2.imwrite('/home/robotics/my/Thesis/Dataset/image/image'+str(d-1)+'.jpg', distorted)
# 			cv2.imwrite('/home/robotics/my/Thesis/Dataset/image_undistorted/undistorted'+str(d-1)+'.jpg', image)
# 			d=d+1
# 			mode=0
# 			break
# 		elif k==ord('<'):
# 			mode = 0
# 		elif k==ord('y'):
# 			mode = 1
# 		elif k == ord('x'):
# 			mode = 2
# 		elif k==ord('c'):
# 			mode = 3
# 		elif k==ord('v'):
# 			mode = 4
# 		elif k==ord('b'):
# 			mode = 5





a =cv2.imread('/home/robotics/my/right/right/binary/image115.jpg',0)
b = cv2.imread('/home/robotics/my/right/right/frame115.jpg')

d=2251#322
for i in range(2251,len(imgs)):
	print(f"file number: {d}, sequence number: {i}")

	circle_size=10
	undis = '/home/robotics/my/Thesis/Dataset/drive/a/lanes/image'+str(d)+'.jpg'
	bina  = '/home/robotics/my/Thesis/Dataset/drive/b/lanes/image'+str(d)+'.png'
	mask  = '/home/robotics/my/Thesis/Dataset/drive/mask/image'+str(d)+'.png'
	image = cv2.imread(undis)
	trans_mask = cv2.imread(mask)
	c = cv2.imread(bina,0)
	res = cv2.bitwise_and(image, image, mask=c)
	# cv2.imshow('gdd',res)
	# cv2.imshow('g',a)
	#plt.show()
	cv2.namedWindow('dd')
	cv2.setMouseCallback('dd', click_event)

	while(1):
		res = cv2.bitwise_and(image, image, mask=c)
		f=np.dstack((c,c,c))
		check = np.zeros_like(f)
		b = np.nonzero(f)
		check[b[0],b[1]]=(0,255,0)
		trans_mask = cv2.addWeighted(image, 1, check, 0.3, 0)
		cv2.imshow('dd',res)
		cv2.moveWindow('dd',20,20)
		cv2.imshow('ff',c)
		cv2.moveWindow('ff',800,20)
		cv2.imshow('f',image)
		cv2.moveWindow('f',20,600)
		cv2.imshow('gds',trans_mask)
		cv2.moveWindow('gds',800,600)
		k = cv2.waitKey(1)
		if k == ord('s'):
			# a=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY)
			# binary = np.zeros_like(a)
			# binary[img!=0]=1
			cv2.imwrite(bina, c)
			cv2.imwrite(mask, trans_mask)
			cv2.imwrite(undis, image)
			d=d+1
			mode=0
			break
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
		# elif k==ord('q'):
		# 	mode = 6




# a3 = np.array( [[[10,10],[100,10],[100,100],[10,100]]], dtype=np.int32 )
# im = np.zeros([240,320],dtype=np.uint8)
# cv2.fillPoly( im, a3, 255 )
# print(a3)