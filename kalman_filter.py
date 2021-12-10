from math import e
import matplotlib.pyplot as plt
import time
import numpy as np
from tdmclient import ClientAsync

j = 0 #Auxiliary variable
Ts = 0.01 #Sampling time for  now set to motor speed update time
theta = 0.0 #rad
vision = 0 #Set to 1 if position from camera available

speed_conv_factor = 0.347 #Speed from thymio speed to mm/s (to confirm)
rot_conv_factor = 0.0066

P_est_prev = 10 * np.identity(6) #Initial P_est
x_est_prev = np.array([[0], [0], [0], [0], [0], [0]]) #Initial position
x_est_range = x_est_prev #Matrix storing all values from the beginning

def kalman_filter(Ts, speed_x, speed_y, speed_w ,x_est_prev, P_est_prev, vision=0, pos_x_m=0.0, pos_y_m=0.0, theta_m=0):
    
    N = 6
    A = np.array([[1, Ts, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, Ts, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, Ts], [0, 0, 0, 0, 0, 1]])
    #State-space model (with acceleration, C to be modified as well)
    #A = np.array([[1, Ts, Ts*Ts/2.0, 0, 0, 0, 0, 0], [0, 1, Ts, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 1, Ts, Ts*Ts/2.0, 0, 0], [0, 0, 0, 0, 1, Ts, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 1, Ts], [0, 0, 0, 0, 0, 0, 0, 1]])
    #N = 8

    Q = 6.1*np.identity(N)

    if vision==1:
        y = np.array([[pos_x_m], [pos_y_m], [theta_m], [speed_x], [speed_y], [speed_w]], dtype=float)
        M = 6
        C = np.array([[1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]])
    else:
        y = np.array([[speed_x], [speed_y], [speed_w]], dtype=float)
        M = 3
        C = np.array([[0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]])

    R = 6.1*np.identity(M) #To update in case of measurement errors

    x_est_a_priori = np.dot(A, x_est_prev); #A priori estimate
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T)) + Q
    
    i = y - np.dot(C, x_est_a_priori) #Measurement residual
    S = np.dot(C, np.dot(P_est_a_priori, C.T)) + R
    K = np.dot(P_est_a_priori, np.dot(C.T, np.linalg.inv(S))) #Kalman gain
    x_est = x_est_a_priori + np.dot(K,i) #A posteriori estimate
    P_est = P_est_a_priori - np.dot(K,np.dot(C, P_est_a_priori))
     
    return x_est, P_est

def update_position(Ts, node, variables):
    try:
        global pos_x, pos_y, theta, vision

        #accel_x = 0.00981/23*acc[1]*np.cos(theta)+0.00981/23*acc[0]*np.sin(theta)
        #accel_y = 0.00981/23*acc[1]*np.sin(theta)-0.00981/23*acc[0]*np.cos(theta)
        motor_left_speed = variables["motor_left_speed"]
        motor_right_speed = variables["motor_right_speed"]
        speed_x = (speed_conv_factor*(motor_left_speed+motor_right_speed)/2.0) * np.cos(theta)
        speed_y = (speed_conv_factor*(motor_left_speed+motor_right_speed)/2.0) * np.sin(theta)
        speed_w = rot_conv_factor*(motor_right_speed-motor_left_speed)/(2.0)
        vision = 0
        pos_x_m = 0.0 #TO MODIFY
        pos_y_m = 0.0 #TO MODIFY
        theta_m = 0.0 #TO MODIFY
        x_est, P_est = kalman_filter(Ts, speed_x, speed_y, speed_w ,x_est_prev, P_est_prev, vision, pos_x_m, pos_y_m, theta_m)
        theta = x_est[4]
        x_est_range = np.concatenate((x_est_range, x_est), axis=1) #Storing all estimations from the beggining
        x_est_prev = x_est
        P_est_prev = P_est
        time.sleep(Ts)
    except KeyError:
        pass

with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(update_position)
            # Will sleep forever:
            await client.sleep()
    client.run_async_program(prog)