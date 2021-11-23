#MICRO-452: Basics of mobile robotics projeect
#Kalman filter algorithm in 1 direction
#Author: Daniel Tataru
#Date: 20.11.21
#V1
#Last modified: 20.11.21

from tdmclient import ClientAsync
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np
from tqdm import tqdm


thymio_to_mms = 0.43478260869565216


qv = 6.15
rv = 6.15

Ts = 0.2

A = np.array([[1, Ts], [0, 1]])
C = np.array([0, 1.0/Ts])

Q = np.array([[qp, 0], [0, q_nu]])
speed_conv_factor = 0.3375



def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.set_variables(motors(0, 0))
            await client.sleep(2)
            await node.set_variables(motors(0, 0))
    client.run_async_program(prog)


def kalman_filter(speed, ground_prev, ground, pos_last_trans, x_est_prev, P_est_prev,
                HT=None, HNT=None, RT=None, RNT=None):

#Estimates the current state using input sensor data and the previous state

#param speed: measured speed (Thymio units)
#param ground_prev: previous value of measured ground sensor
#param ground: measured ground sensor
#param pos_last_trans: position of the last transition detected by the ground sensor
#param x_est_prev: previous state a posteriori estimation
#param P_est_prev: previous state a posteriori covariance

#return pos_last_trans: updated if a transition has been detected
#return x_est: new a posteriori state estimation
#return P_est: new a posteriori state covariance

## Prediciton through the a priori estimate
# estimated mean of the state
    x_est_a_priori = np.dot(A, x_est_prev)

# Estimated covariance of the state
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T))
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori

## Update         
# y, C, and R for a posteriori estimate, depending on transition
    # no transition, use only the speed
    y = speed*speed_conv_factor
    H = np.array([[0, 1]])
    R = r_nu

# innovation / measurement residual
    i = y - np.dot(H, x_est_a_priori)
# measurement prediction covariance
    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R
            
# Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))


# a posteriori estimate
    x_est = x_est_a_priori + np.dot(K,i)
    P_est = P_est_a_priori - np.dot(K,np.dot(H, P_est_a_priori))
    
    return pos_last_trans, x_est, P_est



x_est = [np.array([[0], [0]])]
P_est = [1000 * np.ones(2)]
pos_trans = [0]
k0 = 55
ground = [avg_ground[k0-1]]
speed = [avg_speed[k0-1]]

for k in tqdm(range(55, len(thymio_data))):
    speed.append(avg_speed[k])
    ground.append(avg_ground[k])
    new_pos_last_trans, new_x_est, new_P_est = kalman_filter(speed[-1], ground[-2], ground[-1], 
                                                             pos_trans[-1], x_est[-1], P_est[-1])
    x_est.append(new_x_est)
    P_est.append(new_P_est)
    pos_trans.append(new_pos_last_trans)
