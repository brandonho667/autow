import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys


def show_image(name, img):  # function for displaying the image
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)


# Apply Gaussian Blur --> Reduce noise and smoothen image
def gauss(image):
    return cv2.GaussianBlur(image, (7, 7), 0)

  # outline the strongest gradients in the image --> this is where lines in the image are


def canny(image):
    return cv2.Canny(image, 50, 150)


def region(image):
    height, width, unused = image.shape
    # isolate the gradients that correspond to the lane lines
    rectangle = np.array([
        [(0, height), (0, height//2), (width, height//2), (width, height)]
    ])
    # create a black image with the same dimensions as original image
    mask = np.zeros_like(image)
    # create a mask (triangle that isolates the region of interest in our image)
    mask = cv2.fillPoly(mask, rectangle, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask


def crop_bottom_half(image):  # Delete Top half of image
    cropped_img = image[int(image.shape[0]/2):image.shape[0]]
    return cropped_img


name = 'image'
path = 'frame638.jpg'
frame = cv2.imread(path)
frame = cv2.resize(frame, (600, 400))
frame = cv2.flip(frame, -1)
copy = np.copy(frame)
cropped = crop_bottom_half(frame)
frame = gauss(cropped)
frame = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
show_image(name, frame)

# Green Pixel Count
# Values may need to be modified depending on lighting
Green_Lowerbound = (65, 55, 70)
Green_Upperbound = (75, 85, 95)
Green_mask = cv2.inRange(frame, Green_Lowerbound, Green_Upperbound)
Green_count = np.count_nonzero(Green_mask)
print('Green count:', Green_count)
show_image(name, Green_mask)

# Blue Pixel Count
# Values may need to be modified depending on lighting
Blue_Lowerbound = (100, 130, 75)
Blue_Upperbound = (115, 145, 99)
Blue_mask = cv2.inRange(frame, Blue_Lowerbound, Blue_Upperbound)
Blue_count = np.count_nonzero(Blue_mask)
print('Blue count:', Blue_count)
show_image(name, Blue_mask)


def blue_green_steer_direction(frame):

    frame = cv2.resize(frame, (600, 400))
    frame = cv2.flip(frame, -1)
    cropped = crop_bottom_half(frame)
    frame = gauss(cropped)
    frame = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

    # Green Pixel Count
    # Values may need to be modified depending on lighting
    Green_Lowerbound = (65, 55, 70)
    Green_Upperbound = (75, 85, 95)
    Green_mask = cv2.inRange(frame, Green_Lowerbound, Green_Upperbound)
    Green_count = np.count_nonzero(Green_mask)
    # print('Green count:', Green_count)
    # show_image(name, Green_mask)

    # Blue Pixel Count
    # Values may need to be modified depending on lighting
    Blue_Lowerbound = (100, 130, 75)
    Blue_Upperbound = (115, 145, 99)
    Blue_mask = cv2.inRange(frame, Blue_Lowerbound, Blue_Upperbound)
    Blue_count = np.count_nonzero(Blue_mask)
    # print('Blue count:', Blue_count)
    # show_image(name, Blue_mask)

    if Green_count == 0:
        steer = 1  # turn right
    elif Blue_count == 0:
        steer = 0  # turn left
    else:
        steer = Green_count/Blue_count  # not correct

    return steer
