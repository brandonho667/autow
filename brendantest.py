import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys


def show_image(name, img):  # function for displaying the image
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
