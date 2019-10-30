# date 9.3

import cv2
import numpy as np 


if __name__ == '__main__' :

    # Read image, type the image's name here:
    img = cv2.imread("image1.png")
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    print("Select the Region Of Interest (ROI) on the image (click and drag)")

    # Select ROI
    reg = cv2.selectROI(img)
     
    # Crop image
    target_img0 = img[int(reg[1]):int(reg[1]+reg[3]), int(reg[0]):int(reg[0]+reg[2])]
    target_img = hsv_img[int(reg[1]):int(reg[1]+reg[3]), int(reg[0]):int(reg[0]+reg[2])]
    
    H_max = np.amax(target_img[:,:,0])
    H_avg = np.mean(target_img[:,:,0])
    H_min = np.amin(target_img[:,:,0])
    print("H_max = " + str(H_max))
    print("H_avg = " + str(H_avg))
    print("H_min = " + str(H_min))
    
    S_max = np.amax(target_img[:,:,1])
    S_avg = np.mean(target_img[:,:,1])
    S_min = np.amin(target_img[:,:,1])
    print("S_max = " + str(S_max))
    print("S_avg = " + str(S_avg))
    print("S_min = " + str(S_min))
    
    V_max = np.amax(target_img[:,:,2])
    V_avg = np.mean(target_img[:,:,2])
    V_min = np.amin(target_img[:,:,2])
    print("V_max = " + str(V_max))
    print("V_avg = " + str(V_avg))
    print("V_min = " + str(V_min))
    
    # Adjust HSV threshold
    color_h = np.array([H_avg+10,S_max+200,V_max+70])
    color_l = np.array([H_avg-5,S_min+60,V_min+0])

	# Saturation H:0~180 S:0~255 V:0~255
    color_h[0] = np.clip(color_h[0], 0, 180)
    color_h[1] = np.clip(color_h[1], 0, 255)
    color_h[2] = np.clip(color_h[2], 0, 255)
    color_l[0] = np.clip(color_l[0], 0, 180)
    color_l[1] = np.clip(color_l[1], 0, 255)
    color_l[2] = np.clip(color_l[2], 0, 255)    
    print("color_h = " + str(color_h))
    print("color_l = " + str(color_l))
    mask_img = cv2.inRange(hsv_img, color_l, color_h)
    resultHSV = cv2.bitwise_and(img, img, mask = mask_img)
    # Display cropped image
    cv2.imshow("Selected region", target_img0)
    cv2.imshow("Masked image", resultHSV)
    cv2.waitKey(0)
    