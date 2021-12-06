from ipywidgets import interact, interactive, fixed, interact_manual
import ipywidgets as widgets


import matplotlib.pyplot as plt
import time
import numpy as np

Ts = 0.2
A = np.array([[1, Ts, Ts*Ts/2.0, 0, 0, 0, 0, 0], [0, 1, Ts, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 1, Ts, Ts*Ts/2.0, 0, 0], [0, 0, 0, 0, 1, Ts, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 1, Ts], [0, 0, 0, 0, 0, 0, 0, 1]])
N = 8
M = 5
Q = [0.1*np.identity(N)]
speed_conv_factor = 0.4

C = np.array([[0, 1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 1], [0, 1_0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0]])
R = [0.1*np.identity(M)]
P_est = [10 * np.ones(8)]

x_est_prev = np.array([[0], [0], [0], [0], [0], [0], [0], [0]])

def kalman_filter(speed, accel, x_est_prev, P_est_prev):
    
    x_est_a_priori = np.dot(A, x_est_prev);
    
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T));
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori
    
    ## Update         
    # y, C, and R for a posteriori estimate
    y = np.vstack((speed,accel))

    # innovation / measurement residual
    i = y - np.dot(C, x_est_a_priori);
    # measurement prediction covariance
    S = np.dot(H, np.dot(P_est_a_priori, C.T)) + R;
             
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = np.dot(P_est_a_priori, np.dot(C.T, np.linalg.inv(S)));
    
    
    # a posteriori estimate
    x_est = x_est_a_priori + np.dot(K,i);
    P_est = P_est_a_priori - np.dot(K,np.dot(C, P_est_a_priori));
     
    return x_est, P_est



theta = 0 #rad
r = 15 #mm
l = 80 #mm

def obtain_values(theta):
    accel_x = acc[1]
    accel_y = acc[0]
    speed_x = 0.4*(motor_left_speed+motor_right_speed)*r/2.0 * cos(theta)
    speed_y = 0.4*(motor_left_speed+motor_right_speed)*r/2.0 * sin(theta)
    speed_w = (motor_left_speed-motor_right_speed)*r/(2.0*l)
    speed = np.append(speed_x, speed_y, speed_w)
    speed = np.transpose(speed)
    accel = np.append(accel_x, accel_y)
    return speed, accel