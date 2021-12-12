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
from kalman_filter import update_position
from vision import *

theta = 0
thym_pos = Point(0,0)
motor_left_speed = 0
motor_right_speed = 0
path = []
global_polys = []
compute_global = True
global_goal = Point(MAP_X_MM_SIZE/2,MAP_Y_MM_SIZE/2)


# Main loop
with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)

            while 1:

                # Defining used variables
                global path, global_polys, compute_global, compute_goal, motor_left_speed, motor_right_speed, thym_pos, theta
                kalman_vision = 1
                prox = [i for i in node["prox.horizontal"]]
                del prox[6]
                del prox[5]

                # Retrieve current image from camera feed
                cap = cv2.VideoCapture(0)
                cv2.namedWindow("Thymio field", cv2.WINDOW_NORMAL)
                ret, img = cap.read()
                # Resize image to get mm values and apply gaussian filter for noise reduction
                model_image_size = (MAP_X_MM_SIZE, MAP_Y_MM_SIZE)
                img = cv2.resize(img, model_image_size, interpolation = cv2.INTER_CUBIC)
                img = cv2.GaussianBlur(img,(5,5),0)

                # Creation of the masks
                img_yellow=color_detection(img,'green')    # obstacles
                img_purple=color_detection(img,'purple')    # end goal
                img_blue=color_detection(img,'blue')        # thymio

                # Detecting the end goal position on the map
                polys = image_2_vertices(img_purple,EPSILON_GOAL)
                goal = find_destination(polys, global_goal)
                if math.sqrt((global_goal.y-goal.y)**2+(global_goal.x-goal.x)**2) > FINAL_GOAL:
                    global_goal.x = goal.x
                    global_goal.y = goal.y
                    compute_global = True

                # Detecting Thymio's position on the map and updating it with the kalman values
                polys = image_2_vertices(img_blue,EPSILON_THYMIO)
                thym_pos, theta, kalman_vision = find_thymio(polys, thym_pos, theta)
                x_est = update_position(Ts, thym_pos.x, thym_pos.y, theta, kalman_vision, motor_left_speed, motor_right_speed)
                thym_pos.x = x_est[0][0]
                thym_pos.y = x_est[2][0]
                theta = x_est[4][0]

                # Detecting the obstacles (if first time computing the map or end goal has moved by a certain treshold FINAL_GOAL)
                if compute_global:
                    global_polys = image_2_vertices(img_yellow,EPSILON_OBSTACLES)

                # Updating the line of the path with current positions
                path = global_path(img, global_polys, thym_pos, global_goal, compute_global, path)
                if len(path) >= 1:
                    path_display = [i for i in path]
                    path_display.insert(0,thym_pos)
                    draw_polygon(img, path_display, (255,0,0), 4, complete=False)

                # Updating the thresholds to reach the next goal
                if len(path) == 1:
                    goal_threshold = FINAL_GOAL
                elif len(path) > 1:
                    goal_threshold = STEP_GOAL
                else:
                    return    # Arrived at destination, stops program

                # Determining source of path planner
                if not(all(p == 0 for p in prox)):
                    next_step = PotField(prox, thym_pos, theta, path[0])    # At least one obstacle in front of the Thymio -> using Local Planner
                else:
                    next_step = [path[0].x,path[0].y]                       # No obstacles -> following Global Planner

                # Drawing Thymio's direction arrow
                draw_polygon(img, [thym_pos, Point(thym_pos.x+math.cos(theta)*200,thym_pos.y+math.sin(theta)*200)], (0,0,255), 4, complete=False)

                # Updating motor speeds
                [motor_left_speed, motor_right_speed] = motion_control(node, Point(thym_pos.x,thym_pos.y), theta, next_step, path, goal_threshold, motor_left_speed, motor_right_speed)

                final_image = img
                cv2.imshow("Thymio field",final_image)
                cv2.waitKey(50)

                compute_global = False

                await client.sleep(0.001)

    client.run_async_program(prog)
