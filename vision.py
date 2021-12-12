import cv2
import numpy as np
import math
from pyvisgraph.graph import Point
from constants import *

# Smooth the polygon borders for cleaner edges
def smoothen_contours(raw_contours,epsilon_cst):

    smooth_contours = []

    for cnt in raw_contours:
        if cv2.contourArea(cnt) >= THRESHOLD_AREA:
            hull = cnt
            # perimeter = cv2.arcLength(cnt,True)
            epsilon = epsilon_cst*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            smooth_contours.append(approx)

    return smooth_contours

# Returns the vertices given the contours of a polygon/obstacle
def get_vertices(image, smooth_contours):
    polys = []
    for cnt in smooth_contours:
        points = []
        for vertices in cnt:
            image = cv2.circle(image, (vertices[0][0], vertices[0][1]), radius=5, color=(255, 0, 0), thickness=2)
            points.append(Point(vertices[0][0], vertices[0][1]))
        polys.append(points)
    return polys

# Computes the contours and the vertices given a filtered image (colored masks)
def image_2_vertices(image,epsilon_cst):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    smooth_contours = smoothen_contours(contours,epsilon_cst)
    cv2.drawContours(image, smooth_contours, -1, (200, 200, 200), 3)
    polys = get_vertices(image, smooth_contours)
    return polys

# Returns the position of the Thymio based on a filtered image (blue mask)
def find_thymio(polys, thym_pos, theta):
    # Check if we detect Thymio's triangle
    if len(polys) == 1:
        while np.size(polys) > 3:
            polys[0].pop()

        if np.size(polys) < 3:
            return thym_pos, theta, 0
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
        thym_pos = Point(Pf.x - PC_PF_DIST*math.cos(theta), Pf.y - PC_PF_DIST*math.sin(theta))
        return thym_pos, theta, 1
    else:
        return thym_pos, theta, 0

# Returns the position of the end goal based on a filtered image (pink mask)
def find_destination(polys, global_goal):

    while np.size(polys) > 4:
        polys[0].pop()
    if np.size(polys) == 4:
        P1 = polys[0][0]
        P2 = polys[0][1]
        P3 = polys[0][2]
        P4 = polys[0][3]
        global_goal_y = (P1.y + P2.y +  P3.y + P4.y)/4
        global_goal_x = (P1.x + P2.x +  P3.x + P4.x)/4
        return Point(global_goal_x, global_goal_y)
    else: return global_goal
