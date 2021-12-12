from math import e
import matplotlib.pyplot as plt
import time
import numpy as np
from tdmclient import ClientAsync
from constants import *


def kalman_filter(Ts, speed_x, speed_y, speed_w ,x_est_prev, P_est_prev, vision=0, pos_x_m=0.0, pos_y_m=0.0, theta_m=0):

    N = 6
    A = np.array([[1, Ts, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, Ts, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, Ts], [0, 0, 0, 0, 0, 1]])
    #State-space model (with acceleration, C to be modified as well)
    #A = np.array([[1, Ts, Ts*Ts/2.0, 0, 0, 0, 0, 0], [0, 1, Ts, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 1, Ts, Ts*Ts/2.0, 0, 0], [0, 0, 0, 0, 1, Ts, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 1, Ts], [0, 0, 0, 0, 0, 0, 0, 1]])
    #N = 8

    Q = 6.1*np.identity(N)
    Q[4][4] = 10.2


    if vision==1:
        y = np.array([[pos_x_m], [pos_y_m], [theta_m], [speed_x], [speed_y], [speed_w]], dtype=float)
        M = 6
        C = np.array([[1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]])
        R = 6.1*np.identity(M) #To update in case of measurement errors
        R[0][0] = 0.5
        R[2][2] = 0.5
        R[4][4] = 0.5
        R[5][5] = 4
    else:
        y = np.array([[speed_x], [speed_y], [speed_w]], dtype=float)
        M = 3
        C = np.array([[0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]])
        print('TRYING TO ESTIMATE THYMIO POSITION')
        R = 6.1*np.identity(M) #To update in case of measurement errors
        R[2][2] = 4


    

    x_est_a_priori = np.dot(A, x_est_prev); #A priori estimate
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T)) + Q

    i = y - np.dot(C, x_est_a_priori) #Measurement residual
    S = np.dot(C, np.dot(P_est_a_priori, C.T)) + R
    K = np.dot(P_est_a_priori, np.dot(C.T, np.linalg.inv(S))) #Kalman gain
    x_est = x_est_a_priori + np.dot(K,i) #A posteriori estimate
    P_est = P_est_a_priori - np.dot(K,np.dot(C, P_est_a_priori))

    return x_est, P_est

def update_position(Ts, pos_x_m, pos_y_m, theta_m, vision, motor_left_speed, motor_right_speed):
    print('ENTRY', pos_x_m, pos_y_m, theta_m, vision, motor_left_speed, motor_right_speed)
    global x_est_prev, P_est_prev

    #accel_x = 0.00981/23*acc[1]*np.cos(theta)+0.00981/23*acc[0]*np.sin(theta)
    #accel_y = 0.00981/23*acc[1]*np.sin(theta)-0.00981/23*acc[0]*np.cos(theta)
    speed_x = (speed_conv_factor*(motor_left_speed+motor_right_speed)/2.0) * np.cos(x_est_prev[4])
    speed_y = (speed_conv_factor*(motor_left_speed+motor_right_speed)/2.0) * np.sin(x_est_prev[4])
    speed_w = rot_conv_factor*(motor_right_speed-motor_left_speed)/(2.0)
    x_est, P_est = kalman_filter(Ts, speed_x, speed_y, speed_w ,x_est_prev, P_est_prev, vision, pos_x_m, pos_y_m, theta_m)
    #x_est_range = np.concatenate((x_est_range, x_est), axis=1) #Storing all estimations from the beggining
    x_est_prev = x_est
    P_est_prev = P_est
    print('OUTPUT', x_est)

    return x_est


