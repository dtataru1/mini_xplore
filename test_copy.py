import numpy as np
import cv2
import matplotlib.pyplot as plt
from constants import *

# Reading the input image
#filename ='Images/test11.jpg'
#img = cv2.imread(filename)

cap = cv2.VideoCapture(1)
cv2.namedWindow("Hsv detect", cv2.WINDOW_NORMAL)
ret, img = cap.read()


model_image_size = (MAP_X_MM_SIZE, MAP_Y_MM_SIZE)
img = cv2.resize(img, model_image_size, interpolation = cv2.INTER_CUBIC)
#img = img.astype(np.float32)
#img /= 255.

cv2.imshow("test",img)
cv2.waitKey()
