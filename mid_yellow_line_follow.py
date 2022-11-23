import cv2 as cv2 #importing the library
import numpy as np
import matplotlib.pyplot as plt
import os
import imutils
import argparse

def show_image(name,img): #function for displaying the image
    cv2.imshow(name,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def region_of_interest(image): #function for extracting region of interest
    #bounds in (x,y) format
    bounds = np.array([[[0,500],[0,290],[650,290],[650,550]]],dtype=np.int32)
    # bounds = np.array([[[0,image.shape[0]],[0,image.shape[0]/2],[900,image.shape[0]/2],[900,image.shape[0]]]],dtype=np.int32)
    mask=np.zeros_like(image)
    cv2.fillPoly(mask,bounds,[255,255,255])
    # show_image('inputmask',mask)
    masked_image = cv2.bitwise_and(image,mask)
    show_image('mask',masked_image) 
    return masked_image

#Importing and adjusting image
image = cv2.imread('track video data/frame289.jpg')
image = cv2.resize(image,(650,500))
image = cv2.flip(image,0)
image = region_of_interest(image)

#Convert to hsv color space
lane_image = np.copy(image)
print(lane_image.shape)
lane_image = cv2.cvtColor(lane_image,cv2.COLOR_BGR2HSV)
show_image('hsv_input',lane_image)


#Defining upper and lower boundaries for yellow color
#Create mask 
lower_yellow = np.array([22,24,77])
upper_yellow = np.array([39,164,203])
lane_yellow_mask = cv2.inRange(lane_image,lower_yellow,upper_yellow)
show_image('BW lines',lane_yellow_mask)


result = cv2.bitwise_and(image, image, mask=lane_yellow_mask)
show_image('yellow lines',result)


## Converting to binary threshold and finding contours
gray = cv2.cvtColor(result,cv2.COLOR_RGB2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
# apply binary thresholding apply Otsu's automatic thresholding which automatically determines the best threshold value
ret, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
# visualize the binary image
show_image('Binary image', thresh)
# contours, hierarchy = cv2.findContours(lane_yellow_mask, 1, cv2.CHAIN_APPROX_NONE)
contours, hierarchies = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
for contour in contours:
    cv2.drawContours(result, contour, -1, (0, 255, 0), 3)
show_image('Image with contours',result)


##Finding center of contours
for i in contours:
	M = cv2.moments(i)
	if M['m00'] != 0:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		cv2.drawContours(image, [i], -1, (0, 255, 0), 2)
		cv2.circle(image, (cx, cy), 7, (0, 0, 255), -1)
		cv2.putText(image, "center", (cx - 20, cy - 20),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
	print(f"x: {cx} y: {cy}")
show_image('final image',image)


# #Defining upper and lower boundaries for white color
# lower_white_hls = np.uint8([  0, 170,   0])
# upper_white_hls = np.uint8([5, 250, 255])
# # #Using bitwise operators to segment out white colors
# lane_white_mask = cv2.inRange(lane_image,lower_white_hls,upper_white_hls)
# cv2.imshow('whitemask',lane_white_mask)
# kernel = np.ones((15,15),np.uint8)
# lane_image_2 = cv2.morphologyEx(lane_white_mask, cv2.MORPH_CLOSE, kernel)
# cv2.imshow('hsv_input',lane_image_2)
# cv2.waitKey(0)
# kerenel_dilate = np.ones((7,7),np.uint8)
# lane_image_3 = cv2.dilate(lane_image_2,kerenel_dilate,iterations = 1)
# cv2.imshow('hsv_input',lane_image_3)
# cv2.waitKey(0)


# lower_white_hls = np.uint8([  0, 170,   0])
# upper_white_hls = np.uint8([5, 230, 255])
# lane_white_mask = cv2.inRange(lane_image_2,lower_white_hls,upper_white_hls)
# show_image('whitemask',lane_white_mask)