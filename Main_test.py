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
        #print(cv2.contourArea(cnt))
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

def get_vertices(image, smooth_contours):
    polys = []
    for cnt in smooth_contours:
        points = []
        for vertices in cnt:
            #print(vertices)
            image = cv2.circle(image, (vertices[0][0], vertices[0][1]), radius=5, color=(255, 0, 0), thickness=2)
            points.append(Point(vertices[0][0], vertices[0][1]))
        polys.append(points)
    #print(polys)
    return polys

def image_2_vertices(image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    smooth_contours = smoothen_contours(contours)
    cv2.drawContours(image, smooth_contours, -1, (0, 255, 0), 3)
    polys = get_vertices(image, smooth_contours)

    return polys

def find_thymio(polys):
    global THYMIO_TRIANGLE_TRESHOLD
    # Check if we detect Thymio's triangle
    if np.size(polys) == 3:

        # Find Pf
        P1 = polys[0][0]
        P2 = polys[0][1]
        P3 = polys[0][2]

        if math.sqrt( ((P1.x-P2.x)**2)+((P1.y-P2.y)**2) ) < THYMIO_TRIANGLE_TRESHOLD:
            Pf = P3
        elif math.sqrt( ((P1.x-P3.x)**2)+((P1.y-P3.y)**2) ) < THYMIO_TRIANGLE_TRESHOLD:
            Pf = P2
            P2 = P3
            #P1 = P1
        else:
            Pf = P1
            P1 = P3
            #P2 = P2

        # Find theta
        theta = math.atan2( Pf.y - (P1.y + P2.y) / 2, Pf.x - (P1.x + P2.x) / 2 )

        # Find Pc
        Pc = Point(Pf.x - PC_PF_DIST*math.cos(theta), Pf.y - PC_PF_DIST*math.sin(theta))
        return Pc

    else:
        return Point(-1.0,-1.0)

while (1):

    thymio_position = THYMIO_POSITION
    cam_kalman_enable = 1

    # Extract image
    ret, img = cap.read()
    img = img[::-1,:,:]
    # Resize image to get mm values
    model_image_size = (MAP_X_MM_SIZE, MAP_Y_MM_SIZE)
    img = cv2.resize(img, model_image_size, interpolation = cv2.INTER_CUBIC)

    img_yellow=color_detection(img,'yellow')
    img_purple=color_detection(img,'purple')
    img_blue=color_detection(img,'blue')

    image = img_blue
    polys = image_2_vertices(image)
    thymio_position = find_thymio(polys)
    print(thymio_position)

    if thymio_position == Point(-1.0,-1.0):
        cam_kalman_enable = 0

    else:
        cam_kalman_enable

        image = img_yellow
        polys = image_2_vertices(image)
        path = global_path(polys, thymio_position, END_POSITION)

        ###### TRANSMIT PATH TO BASTIEN

    plt.imshow(image,origin = 'lower')
    plt.show()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
