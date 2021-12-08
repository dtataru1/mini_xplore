import cv2
import numpy as np
import matplotlib.pyplot as plt
from color_detection import *

filename ='test11.jpg'

#Lire l'image
img = cv2.imread(filename, cv2.IMREAD_COLOR)


img_yellow=color_detection(img,'yellow')
img_purple=color_detection(img,'purple')
img_blue=color_detection(img,'blue')



# contours_yellow, hierarchy = cv2.findContours(img_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# cnt = contours_yellow[4]
# cv2.drawContours(img, [cnt], 0, (0,255,0), 3)


fig, ax = plt.subplots(2,2, figsize=(10,10))
ax[0,0].imshow(img[:,:,::-1])
ax[0,1].imshow(img_yellow, cmap='gray')
ax[1,0].imshow(img_purple, cmap='gray')
ax[1,1].imshow(img_blue, cmap='gray')

plt.show()
