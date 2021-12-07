from ipywidgets import interact, interactive, fixed, interact_manual
import ipywidgets as widgets


import matplotlib.pyplot as plt
import time
import numpy as np


# Initializing state space model

Ts = 0.2
A = np.array([[1, Ts, Ts*Ts/2.0, 0, 0, 0, 0, 0], [0, 1, Ts, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 1, Ts, Ts*Ts/2.0, 0, 0], [0, 0, 0, 0, 1, Ts, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 1, Ts], [0, 0, 0, 0, 0, 0, 0, 1]])
N = 8
M = 5
Q = 0.1*np.identity(N)
speed_conv_factor = 0.4


C = np.array([[0, 1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 1], [0, 1_0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0]])
R = 0.1*np.identity(M)
P_est_prev = 10 * np.identity(N)

x_est_prev = np.array([[0], [0], [0], [0], [0], [0], [0], [0]])
x_pos_est = x_est_prev

def kalman_filter(speed, accel, x_est_prev, P_est_prev):
    
    x_est_a_priori = np.dot(A, x_est_prev)
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T))
    
    
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori
    ## Update         
    # y, C, and R for a posteriori estimate
    y = np.vstack((speed,accel))

    # innovation / measurement residual
    i = y - np.dot(C, x_est_a_priori);
    # measurement prediction covariance
    S = np.dot(C, np.dot(P_est_a_priori, C.T)) + R
             
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = np.dot(P_est_a_priori, np.dot(C.T, np.linalg.inv(S)))
    
    
    # a posteriori estimate
    x_est = x_est_a_priori + np.dot(K,i);
    P_est = P_est_a_priori - np.dot(K,np.dot(C, P_est_a_priori));
     
    return x_est, P_est



theta = 0.0 #rad
r = 15.0 #mm
l = 80.0 #mm

speed = np.array([[0.0], [0.0], [0.0]])
accel = np.array([[0.0], [0.0]])
for i in range(2):
    x_est, P_est = kalman_filter(speed, accel, x_est_prev, P_est_prev)
    theta = x_est[6]
    x_pos_est = np.append(x_pos_est, x_est)
    time.sleep(1)
    

