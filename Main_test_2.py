import cv2
import numpy as np
import matplotlib.pyplot as plt
from color_detection import *
from global_planner import *
from constants import *

### Read the canera feed
#img = cv2.imread('Images/Test15.jpg')
cap = cv2.VideoCapture(1)
cv2.namedWindow("Hsv detect", cv2.WINDOW_NORMAL)

def smoothen_contours(raw_contours):
    global THRESHOLD_AREA, EPSILON_CST

    smooth_contours = []

    for cnt in raw_contours:
        print(cv2.contourArea(cnt))
        if cv2.contourArea(cnt) >= THRESHOLD_AREA:
            hull = cnt
            # perimeter = cv2.arcLength(cnt,True)
            epsilon = EPSILON_CST*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            smooth_contours.append(approx)

    return smooth_contours

def inverse_y_axis (position):
    global MAP_Y_MM_SIZE
    return MAP_Y_MM_SIZE - position

# Go from cm coordinates to pixel coordinates for display, position is of type vg.Point(x,y)
def toPixCoord(position):
    position.x = position.x/CAM_2_MAP
    position.y = position.y/CAM_2_MAP
    return position

def toMapCoord(position):
    position.x = position.x*CAM_2_MAP
    position.y= position.y*CAM_2_MAP
    return position



while (1):

    # Extract image
    ret, img = cap.read()
    img = img[::-1,:,:]
    # Resize image to get mm values
    model_image_size = (MAP_X_MM_SIZE, MAP_Y_MM_SIZE)
    img = cv2.resize(img, model_image_size, interpolation = cv2.INTER_CUBIC)

    img_yellow=color_detection(img,'yellow')
    img_purple=color_detection(img,'purple')
    img_blue=color_detection(img,'blue')

    image = img_yellow

    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    smooth_contours = smoothen_contours(contours)

    cv2.drawContours(image, smooth_contours, -1, (0, 255, 0), 3)

    polys = []

    for cnt in smooth_contours:
        points = []
        for vertices in cnt:
            print(vertices)
            image = cv2.circle(image, (vertices[0][0], vertices[0][1]), radius=5, (255, 0, 0), thickness=2)
            points.append(Point(vertices[0][0], vertices[0][1]))
        polys.append(points)
    print(polys)

    path = global_path(polys, THYMIO_POSITION, END_POSITION)

    ###### TRANSMIT PATH TO BASTIEN

    plt.imshow(image,origin = 'lower')
    plt.show()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
