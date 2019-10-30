#!/usr/bin/env python

import cv2
import numpy as np

"""
To init blob search params, will be init (called) in the ImageConverter class
"""
def blob_search_init():

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	################# Your Code Start Here #################

	# Filter by Color
	params.filterByColor = False
	# Filter by Area.
	params.filterByArea = True
	params.minArea = 100
	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.2
	# Filter by Inerita
	params.filterByInertia = False
	# Filter by Convexity
	params.filterByConvexity = False
	# Any other params to set???
	params.minThreshold = 200
	params.maxThreshold = 255


	################## Your Code End Here ##################

	# Create a detector with the parameters
	blob_detector = cv2.SimpleBlobDetector_create(params)

	return blob_detector


"""
To find blobs in an image, will be called in the callback function of image_sub subscriber
"""
def blob_search(image, detector):

	# Convert the color image into the HSV color space
	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


	############################ Your Code Start Here ############################

	# Find lower & upper for orange

	lower =(10,200,155)      # oranger lower
	upper = (20,255,245)   # orange upper

	############################# Your Code End Here #############################


	# Define a mask using the lower and upper bounds of the orange color
	mask_image = cv2.inRange(hsv_image, lower, upper)

	crop_top_row = 160
	crop_bottom_row = 370
	crop_top_col = 250
	crop_bottom_col = 570

	crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

	blob_image_center = []

	############################ Your Code Start Here ############################

	# Call opencv simpleBlobDetector functions here to find centroid of all large enough blobs in
	# crop_image. Make sure to add crop_top_row and crop_top_col to the centroid row and column found

	# Make sure this blob center is in the full image pixels not the cropped image pixels

	im_with_keypoints = image

	retval, threshold = cv2.threshold(mask_image, 200, 255, cv2.THRESH_BINARY_INV)
	# Draw centers on each blob, append all the centers to blob_image_center as string in format "x y"
	keypoints = detector.detect(threshold)

	for i in range(len(keypoints)):
		blob_image_center.append([keypoints[i].pt[0], keypoints[i].pt[1]])

	print("No. of Blobs: " + str(len(blob_image_center)))


	im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	############################# Your Code End Here #############################

	# Draw small circle at pixel coordinate crop_top_col, crop_top_row so you can move a color
	# under that pixel location and see what the HSV values are for that color.
	image = cv2.circle(image, (int(crop_top_col), int(crop_top_row)), 3, (0, 0, 255), -1)
	print('H,S,V at pixel ' + str(crop_top_row) + ' ' + str(crop_top_col) + ' ' + str(hsv_image[crop_top_row,crop_top_col]))

	cv2.namedWindow("Maze Window")
	cv2.imshow("Maze Window", im_with_keypoints)

	cv2.namedWindow("MaskImage Window")
	cv2.imshow("MaskImage Window", mask_image)

	cv2.namedWindow("Crop Window")
	cv2.imshow("Crop Window", crop_image)

	cv2.waitKey(2)

	return blob_image_center
