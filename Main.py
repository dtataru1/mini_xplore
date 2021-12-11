import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from color_detection import color_detection
from pyvisgraph.graph import Point
from global_planner import global_path, draw_polygon
from constants import *
from potential_field import *
from tdmclient import ClientAsync
from local_motion_control import motion_control

global path, global_goal, global_polys, compute_global
path = []
global_polys = []
compute_global = True
global_goal = Point(MAP_X_MM_SIZE/2,MAP_Y_MM_SIZE/2)

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
            #print(vertices)
            image = cv2.circle(image, (vertices[0][0], vertices[0][1]), radius=5, color=(255, 0, 0), thickness=2)
            points.append(Point(vertices[0][0], vertices[0][1]))
        polys.append(points)
    #print(polys)
    return polys

# Computes the contours and the vertices given a filtered image (colored masks)
def image_2_vertices(image,epsilon_cst):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    smooth_contours = smoothen_contours(contours,epsilon_cst)
    cv2.drawContours(image, smooth_contours, -1, (200, 200, 200), 3)
    polys = get_vertices(image, smooth_contours)

    return polys

# Returns the position of the Thymio based on a filtered image (blue mask)
def find_thymio(polys):
    # Check if we detect Thymio's triangle
    while np.size(polys) > 3:
        polys[0].pop()
    if np.size(polys) < 3:
        return Point(-1.0,-1.0), 0
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
    return Pc, theta

# Returns the position of the end goal based on a filtered image (pink mask)
def find_destination(polys):
    global global_goal

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

#cap.release()
#cv2.destroyAllWindows()


with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)

            #await node.wait_for_variables('prox.horizontal')
            while 1:
                global path, global_polys, compute_global, compute_goal

                prox = [i for i in node["prox.horizontal"]]
                #print('prox', prox)
                del prox[6]
                del prox[5]
                #print('prox', prox)

                kalman_vision = 1
                cap = cv2.VideoCapture(1)
                cv2.namedWindow("Thymio field", cv2.WINDOW_NORMAL)
                # Extract image
                ret, img = cap.read()
                # Resize image to get mm values and apply gaussian filter for noise reduction
                model_image_size = (MAP_X_MM_SIZE, MAP_Y_MM_SIZE)
                img = cv2.resize(img, model_image_size, interpolation = cv2.INTER_CUBIC)
                img = cv2.GaussianBlur(img,(5,5),0)

                # Creation of the masks
                img_yellow=color_detection(img,'yellow')    # obstacles
                img_purple=color_detection(img,'purple')    # end goal
                img_blue=color_detection(img,'blue')        # thymio

                # Detecting the end goal position on the map
                polys = image_2_vertices(img_purple,EPSILON_GOAL)
                global_goal = find_destination(polys)

                # Detecting Thymio's position on the map
                polys = image_2_vertices(img_blue,EPSILON_THYMIO)
                #print('polys', polys)
                thym_pos, theta = find_thymio(polys)
                #print('thym_pos', thym_pos)

                if thym_pos == Point(-1.0,-1.0): # Vision is obstructed, we can't observe the obstacles
                    kalman_vision = 0
                    #print('WE LOST THYMIO...')
                    #polys = polys_backup

                ### DANIEL KALMAN ###
                #thym_pos = kalman(thym_pos, kalman_vision)

                if compute_global: # first time computes the map
                    # Detecting the obstacles
                    global_polys = image_2_vertices(img_yellow,EPSILON_OBSTACLES)

                path = global_path(img_yellow, global_polys, thym_pos, global_goal, compute_global, path)

                if len(path) >= 1:
                    path_display = [i for i in path]
                    path_display.insert(0,thym_pos)
                    draw_polygon(img_yellow, path_display, (100,100,0), 4, complete=False)

                #print('path', path)
                #print(path)
                if len(path) == 1:
                    goal_threshold = FINAL_GOAL

                elif len(path) > 1:
                    goal_threshold = STEP_GOAL

                else:
                    return

                #print(thym_pos, theta, path, goal_threshold)
                ###### TRANSMIT PATH TO LOCAL PLANNER

                #print(thym_pos.x,' ',thym_pos.y,' ', theta,' ', path[0].x,' ', path[0].y)
                next_step = PotField(prox, thym_pos, theta, path[0])
                #print('next_step', next_step)

                draw_polygon(img_yellow, [thym_pos, Point(next_step[0]*10, next_step[1]*10)], (100,100,0), 4, complete=False)
                #img_yellow = cv2.circle(img_yellow, (int(next_step[0]), int(next_step[1])), radius=20, color=(0, 0, 255), thickness=4)

                print('thym_pos.x', thym_pos.x,'thym_pos.y',thym_pos.y,'theta', theta,'next_step[0]', next_step[0],'next_step[1]', next_step[1])

                motion_control(node, Point(thym_pos.x,thym_pos.y), theta, next_step, path, goal_threshold)
                #motion_control(node, variables, Point(thym_pos.x,thym_pos.y), theta, [path[0].x,path[0].y], path, goal_threshold)

                final_image = img_purple | img_blue | img_yellow
                #final_image = img_blue

                cv2.imshow("Thymio field",final_image)
                cv2.waitKey(50)

                compute_global = False

                await client.sleep(0.1)

    client.run_async_program(prog)
