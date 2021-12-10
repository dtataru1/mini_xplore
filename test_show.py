import cv2
import numpy as np
import matplotlib.pyplot as plt
from color_detection import *
from global_planner import *
from constants import *
from potential_field import *
from tdmclient import ClientAsync
from local_motor_control import motion_control
import time

### Read the canera feed
#img = cv2.imread('Images/Test15.jpg')
cap = cv2.VideoCapture(1)
cv2.namedWindow("Hsv detect", cv2.WINDOW_NORMAL)

def main(node, variables):

    try:
    # Extract image
    #while 1:
        ret, img = cap.read()
        #img = img[::-1,:,:]
        # Resize image to get mm values
        model_image_size = (MAP_X_MM_SIZE, MAP_Y_MM_SIZE)
        img = cv2.resize(img, model_image_size, interpolation = cv2.INTER_CUBIC)

        img_yellow=color_detection(img,'yellow')
        img_purple=color_detection(img,'purple')
        img_blue=color_detection(img,'blue')

        image = img_blue
        cv2.line(image, [10, 550], [200, 400], (255,0,0), thickness=2)

        cv2.imshow('Normal Video',image)
        #cv2.show()
        #cv2.waitKey(1)
        #plt.imshow(image,origin = 'lower')
        #plt.show()

        #plt.pause(5) # pause how many seconds
        #plt.close()

        cv2.waitKey(100)
        #break

        #cap.release()
        #cv2.destroyAllWindows()
        #time.sleep(1)
    except KeyError:
        pass  # prox.horizontal not found

with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(main)
            # Will sleep forever:
            await client.sleep()
    client.run_async_program(prog)
