#!/usr/bin/env python

import cv2
import numpy as np
import math
# ============================= Student's code starts here ===================================

# Params for camera calibration
theta = -0.0263097172529
beta =  770.259696466
tx = -0.261830861284
ty = -0.04014532872

# Function that converts image coord to world coord
def IMG2W(x,y):
	# xyw = []

	A = np.array([[x/beta], [y/beta]])
	Rz = np.array([[math.cos(theta), -1*math.sin(theta)], \
        [math.sin(theta), math.cos(theta)]])
	B = Rz.dot(A) + np.array([[tx], [ty]])

	xw = B[1,0]
	yw = B[0,0]
	xyw = [xw,yw]
	# xyw.append([xw,yw])
	# print(xyw)
	return xyw


def blob_search(image_raw, color):

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	# Filter by Color 
	params.filterByColor = False

	# Filter by Area.
	params.filterByArea = True
	params.minArea = 100  


	# Filter by Circularity
	params.filterByCircularity = False


	# Filter by Inerita
	params.filterByInertia = False


	# Filter by Convexity
	params.filterByConvexity = False

	params.minThreshold = 200
	params.maxThreshold = 255

	# Create a detector with the parameters
	detector = cv2.SimpleBlobDetector_create(params)

	# Crop the image
	image = image_raw[10:400, 10:540].copy()

	# Convert the image into the HSV color space
	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	if (color == "red"):
		lower = (0,160,160) 		#pink
		upper = (10,255,255)

		# lower = (0,200,200) 		#pink
		# upper = (10,255,255)
	elif (color == "green"):
		lower = (45,130,110)     # green
		upper = (65,230,185)

		# lower = (45,150,130)     # green
		# upper = (65,210,165)


	# Define a mask using the lower and upper bounds of the target color 
	mask_image = cv2.inRange(hsv_image, lower, upper)

	# im_with_keypoints = image

	# retval, threshold = cv2.threshold(mask_image, 200, 255, cv2.THRESH_BINARY_INV)

	keypoints = detector.detect(mask_image)
	# keypoints = detector.detect(threshold)

	#im_with_keypoints = cv2.drawKeypoints(image, keypoints, ???)
	im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	# Find blob centers in the image coordinates
	blob_image_center = []

	x1 = 0
	y1 = 0
	x2 = 0
	y2 = 0

	# for i in keypoints:
	# 	blob_image_center.append([keypoints[i].pt[0], keypoints[i].pt[1]])
	for i in range(len(keypoints)):
		blob_image_center.append([keypoints[i].pt[0], keypoints[i].pt[1]])


	if(len(blob_image_center) == 0):
		x1 = 0
		# print("No blob found!")
	elif(len(blob_image_center) == 2):
		x1 = int(blob_image_center[0][0])
		y1 = int(blob_image_center[0][1])
		x2 = int(blob_image_center[1][0])
		y2 = int(blob_image_center[1][1])

	xw_yw = []

	if(len(blob_image_center) == 0):
		xw_yw = []
		# print("No block found!")
	else:
		# Convert image coordinates to global world coordinate
		# Hint: use IM2W() function
		[x1w,y1w] = IMG2W(x1,y1)
		[x2w,y2w] = IMG2W(x2,y2)

		xw_yw.append([x1w,y1w])
		xw_yw.append([x2w,y2w])


	# print('xw_yw : ', xw_yw)
	cv2.namedWindow("Maze Window")
	cv2.imshow("Maze Window", im_with_keypoints)

	cv2.namedWindow("Camera View")
	cv2.imshow("Camera View", image)
	
	cv2.namedWindow("MaskImage Window")
	cv2.imshow("MaskImage Window", mask_image)


	cv2.waitKey(2)

	return xw_yw