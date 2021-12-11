import cv2
import numpy as np
import matplotlib.pyplot as plt


def color_detection(img,color):

    hsv_img=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #YELLOW_MIN = np.array([0, 35, 0])
    #YELLOW_MAX = np.array([30, 255, 255])
    #PURPLE_MIN = np.array([150, 100, 120])
    #PURPLE_MAX = np.array([170, 255, 255])
    #BLUE_MIN = np.array([90, 60, 0])
    #BLUE_MAX = np.array([135, 255, 255])

    YELLOW_MIN = np.array([60, 50, 0])
    YELLOW_MAX = np.array([100, 255, 255])
    PURPLE_MIN = np.array([150, 100, 80])
    PURPLE_MAX = np.array([170, 255, 255])
    BLUE_MIN = np.array([90, 60, 70])
    BLUE_MAX = np.array([135, 255, 255])

    if color=='yellow':
        mask=cv2.inRange(hsv_img, YELLOW_MIN, YELLOW_MAX)
    elif color=='purple':
        mask=cv2.inRange(hsv_img, PURPLE_MIN, PURPLE_MAX)
    elif color=='blue':
        mask=cv2.inRange(hsv_img, BLUE_MIN, BLUE_MAX)

    #mask_inv = cv2.bitwise_not(mask)

    return mask
