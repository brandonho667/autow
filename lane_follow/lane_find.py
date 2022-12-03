from utils import vesc
import cv2
import numpy as np
from utils.lane_detect import *
import depthai as dai
import signal

# calc steering for given center point
def calc_angle(frame, center_pt):
    width, height = frame.shape[1], frame.shape[0]
    x, y = center_pt
    x -= width/2
    if abs(x) < 0.01:
        return 0.5
    y = height - y
    angle = np.arctan(y/x)*1.3
    return angle/np.pi + 0.5

def find_intersection(line1, line2):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    a1 = y2-y1
    b1 = x1-x2
    c1 = a1*(x1) + b1*(y1)

    # Line CD represented as a2x + b2y = c2
    a2 = y4-y3
    b2 = x3-x4
    c2 = a2*(x3) + b2*(y3)

    determinant = a1*b2 - a2*b1
    if (determinant == 0):
        # The lines are parallel. This is simplified
        # by returning a pair of FLT_MAX
        return -1
    else:
        x = (b2*c1 - b1*c2)/determinant
        y = (a1*c2 - a2*c1)/determinant
        return x, y

def lane_steer(frame):
    copy = np.copy(frame)
    edges = canny(gauss(hsv(copy)))
    isolated = region(edges)
    # show_image('edges', edges)
    # show_image('isolated', isolated)

    # DRAWING LINES: (order of params) --> region of interest, bin size (P, theta), min intersections needed, placeholder array,
    lines = cv2.HoughLinesP(
        isolated, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    if lines is None or len(lines) < 2:
        return None
    # print(f"lines: {lines}, {lines.shape}")
    averaged_lines = average(copy, lines)
    # print(
    #     f"averaged_lines: {averaged_lines}, {averaged_lines.shape}")

    # print(f"pt 1 type {type(averaged_lines[0][0])}")
    intersection = find_intersection(
        averaged_lines[0], averaged_lines[1])
    if intersection == -1:
        intersection = (copy.shape[1]//2, copy.shape[0]//2)
    # print(f"intersection @ {intersection}")
    return calc_angle(copy, intersection)


def region_of_interest(image): #function for extracting region of interest
    #bounds in (x,y) format
    bounds = np.array([[[0,500],[0,290],[650,290],[650,550]]],dtype=np.int32)
    # bounds = np.array([[[0,image.shape[0]],[0,image.shape[0]/2],[900,image.shape[0]/2],[900,image.shape[0]]]],dtype=np.int32)
    mask=np.zeros_like(image)
    cv2.fillPoly(mask,bounds,[255,255,255])
    # show_image('inputmask',mask)
    masked_image = cv2.bitwise_and(image,mask)
    # show_image('mask',masked_image) 
    return masked_image

def center_steer(image):
    centers = []
    #Importing and adjusting image
    image = cv2.resize(image,(650,500))
    image = cv2.flip(image,0)
    image = region_of_interest(image)

    #Convert to hsv color space
    lane_image = np.copy(image)
    # print(lane_image.shape)
    lane_image = cv2.cvtColor(lane_image,cv2.COLOR_BGR2HSV)
    # show_image('hsv_input',lane_image)


    #Defining upper and lower boundaries for yellow color
    #Create mask 
    lower_yellow = np.array([22,33,100])
    upper_yellow = np.array([39,164,203])
    lane_yellow_mask = cv2.inRange(lane_image,lower_yellow,upper_yellow)
    # show_image('BW lines',lane_yellow_mask)


    result = cv2.bitwise_and(image, image, mask=lane_yellow_mask)
    # show_image('yellow lines',result)


    ## Converting to binary threshold and finding contours
    gray = cv2.cvtColor(result,cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # apply binary thresholding apply Otsu's automatic thresholding which automatically determines the best threshold value
    ret, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # visualize the binary image
    # show_image('Binary image', thresh)
    # contours, hierarchy = cv2.findContours(lane_yellow_mask, 1, cv2.CHAIN_APPROX_NONE)
    contours, hierarchies = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        cv2.drawContours(result, contour, -1, (0, 255, 0), 3)
    # show_image('Image with contours',result)


    ##Finding center of contours
    for i in contours:
        M = cv2.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centers.append((cx,cy))

    if not centers:
        # print("Couldn't find center")
        return None
    low_pt = max(centers, key=lambda x: x[1])
    return calc_angle(image, low_pt)

