#!/usr/bin/env python

import sys
import cv2
import copy
import time
import numpy as np
import math

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lab3_func import blob_search_init, blob_search


######################## Replace values below in Part2 of Lab3 ########################

# Params for camera calibration
theta = 0.0131571354728
beta = 760.56311235
tx = -0.31348615917
ty = -0.0705519480519


#######################################################################################


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.coord_pub = rospy.Publisher("/coord_center", String, queue_size=10)
        self.loop_rate = rospy.Rate(SPIN_RATE)
        self.detector = blob_search_init()

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
        	print("ROS is shutdown!")

    def image_callback(self, data):
    	global theta
    	global beta
    	global tx
    	global ty

    	try:
    		raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	except CvBridgeError as e:
    		print(e)

    	cv_image = cv2.flip(raw_image, -1)
    	cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

    	blob_image_center = blob_search(cv_image, self.detector)

    	if(len(blob_image_center) == 0):
    		print("No blob found!")
    		self.coord_pub.publish("")
    	elif(len(blob_image_center) == 1):
    		x1 = int(blob_image_center[0][0])
    		y1 = int(blob_image_center[0][1])
    		# x2 = int(blob_image_center[1][0])
    		# y2 = int(blob_image_center[1][1])

    		# if x1 > x2:
    		# 	T = [x1,x2, y1, y2]
    		# 	x2 = T[0]
    		# 	x1 = T[1]
    		# 	y2 = T[2]
    		# 	y1 = T[3]

    		A = np.array([[x1/beta], [y1/beta]])
    		Rz = np.array([[math.cos(theta), -1*math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    		B = Rz.dot(A) + np.array([[tx], [ty]])

    		xw = B[1,0]
    		yw = B[0,0]

    		xy_w = str(xw) + str(' ') + str(yw)
    		print(xy_w)
    		self.coord_pub.publish(xy_w)


def main():

    SPIN_RATE = 20 # 20Hz

    rospy.init_node('lab3ImageNode', anonymous=True)

    ic = ImageConverter(SPIN_RATE)

    try:
    	rospy.spin()
    except KeyboardInterrupt:
    	print("Shutting down!")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
