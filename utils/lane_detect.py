import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys


def show_image(name, img):  # function for displaying the image
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def grey(image):
  # convert to grayscale
    image = np.asarray(image)
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

  # Apply Gaussian Blur --> Reduce noise and smoothen image


def gauss(image):
    return cv2.GaussianBlur(image, (7, 7), 0)

  # outline the strongest gradients in the image --> this is where lines in the image are


def canny(image):
    return cv2.Canny(image, 50, 150)


def region(image):
    height, width = image.shape
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


def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    # make sure array isn't empty
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            # draw lines on a black image
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return lines_image


def average(image, lines):
    left = []
    right = []

    width = image.shape[1]
    height = image.shape[0]

    if lines is not None:
        for line in lines:
            print(line)
            x1, y1, x2, y2 = line.reshape(4)
            # fit line to points, return slope and y-int
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            print(parameters)
            slope = parameters[0]
            y_int = parameters[1]
            # lines on the right have positive slope, and lines on the left have neg slope
            if slope < 0:
                left.append((slope, y_int))
            else:
                right.append((slope, y_int))

    left_line = [0, 0, 0, height]
    right_line = [width, 0, width, height]
    if len(left) != 0:
        left_average = np.average(left, axis=0)
        pll = make_points(image, left_average)
        if sum([abs(x) for x in pll]) < 1e5:
            left_line = pll
            print(f"valid left line {left_line}")
    if len(right) != 0:
        right_average = np.average(right, axis=0)
        prl = make_points(image, right_average)
        if sum([abs(x) for x in prl]) < 1e5:
            right_line = prl
            print(f"valid right line {right_line}")

    return np.array([left_line, right_line])


def make_points(image, average):
    print(f"average {average}")

    slope, y_int = average
    y1 = image.shape[0]
    # how long we want our lines to be --> 3/5 the size of the image
    y2 = int(y1 * (3/5))
    # determine algebraically
    x1 = int((y1 - y_int) // slope)
    x2 = int((y2 - y_int) // slope)
    return np.array([x1, y1, x2, y2])
