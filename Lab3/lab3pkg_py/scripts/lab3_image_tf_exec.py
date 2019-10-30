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


# Params for camera calibration
theta = 0
beta = 0
tx = 0
ty = 0

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
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
            # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Flip the image 180 degrees
        cv_image = cv2.flip(raw_image, -1)

        # Draw a black line on the image
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # cv_image is normal color image
        blob_image_center = blob_search(cv_image, self.detector)

        # Given world coordinate (xw, yw)
        xw = 0.2875
        yw = 0.1125

        # Only two blob center are found on the image
        if(len(blob_image_center) == 2):

            x1 = int(blob_image_center[0][0])
            y1 = int(blob_image_center[0][1])
            x2 = int(blob_image_center[1][0])
            y2 = int(blob_image_center[1][1])

            print("Blob Center 1: ({0}, {1}) and Blob Center 2: ({2}, {3})".format(x1, y1, x2, y2))

            ################################# Your Code Start Here #################################

            # Calculate beta, tx and ty, given x1, y1, x2, y2

            d = ( (x1 - x2)**2 + (y1 - y2)**2 )**0.5

            if x1 > x2:
                T = [x1,x2, y1, y2]
                x2 = T[0]
                x1 = T[1]
                y2 = T[2]
                y1 = T[3]

            beta = d/0.1  # Pixels per meter


            theta = math.asin((y2 - y1)/ d)

            Rz = np.array([[math.cos(theta), -1*math.sin(theta)], [math.sin(theta), math.cos(theta)]])


            # Calculate Tx, Ty

            A = Rz.dot(np.array([[x1/beta], [y1/beta]]))

            cam_origin_x = yw - A[0,0]
            cam_origin_y = xw - A[1,0]

            tx = cam_origin_x
            ty = cam_origin_y




            ################################## Your Code End Here ##################################

            print("theta = {0}\nbeta = {1}\ntx = {2}\nty = {3}\n".format(theta, beta, tx, ty))

        else:
            print("No Blob found! ")


def main():

    SPIN_RATE = 20 # 20Hz

    rospy.init_node('lab3ImageCalibrationNode', anonymous=True)

    ic = ImageConverter(SPIN_RATE)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
