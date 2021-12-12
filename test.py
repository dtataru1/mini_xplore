import math
import numpy as np
import pyvisgraph as vg
from pyvisgraph.graph import Point
from global_planner import draw_polygon
import cv2
import pyclipper
from constants import *

size = 15
x = np.arange(-size,size,1)
y = x
X, Y = np.meshgrid(x,y)

#print("X", X, "Y", Y)

def add_map_borders(polys):
    polys.append([vg.Point(0,200), vg.Point(0,590), vg.Point(-10,590), vg.Point(-10,200)])
    polys.append([vg.Point(200,0), vg.Point(1230,0), vg.Point(1230,-10), vg.Point(200,-10)])
    polys.append([vg.Point(1430,200), vg.Point(1430,580), vg.Point(1440,580), vg.Point(1440,200)])
    polys.append([vg.Point(200,790), vg.Point(1230,790), vg.Point(1230,800), vg.Point(200,800)])

polys = []
polys.append([vg.Point(100,100), vg.Point(150,100), vg.Point(150,150), vg.Point(125,150), vg.Point(125,125), vg.Point(100,125)])
#polys.append([vg.Point(0,0), vg.Point(0,0), vg.Point(0,100), vg.Point(0,690)])
#polys.append([vg.Point(0,100), vg.Point(0,690)])
#polys.append([vg.Point(100,0), vg.Point(1330,0)])
#polys.append([vg.Point(1430,100), vg.Point(1430,680)])
#polys.append([vg.Point(100,790), vg.Point(1330,790)])

# TEST 1

#print(len(polys))
#for p in polys[len(polys)-4:]:
#    print(p)

# TEST 2

#obst_RT = np.array([math.sqrt(2), -math.sqrt(2), 1])

#thym_RG =  np.array([5, 4])
#thym_RG_angle = -math.pi/4

# https://theailearner.com/tag/geometric-transformation-numpy/
#c, s = np.cos(thym_RG_angle), np.sin(thym_RG_angle)
#R = np.array([[c, -s, thym_RG[0]*(1-c)+thym_RG[1]*s],
#              [s, c, thym_RG[1]*(1-c)-thym_RG[0]*s]])

#a = np.array([[1, 0],
#              [1, 1]])
#b = np.array([4, 1])

#print(np.matmul(a, b))
#print(np.matmul(thym_RG,R))
#print(np.matmul(R,obst_RT))
#obst_RT = thym_RG + np.multiply(R, thym_RG)

# TEST 3

#path = [Point(450,45), Point(500,55)]
#local_goal = Point(120,45)
#path.insert(0,local_goal)
#print(path)

# TEST 4
#for p in ([0, 1, 3, 4]):
#    print(p)

# TEST 5


#pos = Point(50.00,25.00)

#x = np.arange(pos.x-size,pos.x+size,1)
#y = np.arange(pos.y-size,pos.y+size,1)
#X, Y = np.meshgrid(x,y)

#dx = np.zeros_like(X)
#dy = np.zeros_like(Y)

#print(x,y,X,Y)


#A = B = C = 1

#A += 1

#print(A, B, C)

#print(polys)
#rint(np.size(polys))
#print(len(polys))

def increase_coordinates(polygon):
    subj = []
    for p in polygon:
        subj.append((math.floor(p.x),math.floor(p.y)))

    pco = pyclipper.PyclipperOffset()
    pco.AddPath(subj, pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)

    solution = pco.Execute(OBST_INCREASE)
    polygon = []
    for i in range(len(solution[0])):
        polygon.append(Point(solution[0][i][0],solution[0][i][1]))
    return polygon

new_polys = increase_coordinates(polys[0])

cap = cv2.VideoCapture(0)
cv2.namedWindow("Thymio field", cv2.WINDOW_NORMAL)
# Extract image
ret, img = cap.read()
draw_polygon(img, polys[0], (0,0,255), 4, complete=True)
draw_polygon(img, new_polys[0], (0,0,255), 4, complete=True)
cv2.imshow("Thymio field",img)
cv2.waitKey(50000)
