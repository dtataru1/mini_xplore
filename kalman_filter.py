#MICRO-452: Basics of mobile robotics projeect
#Kalman filter algorithm
#Author: Daniel Tataru
#Date: 18.11.21
#V2
#Last modified: 20.11.21


import numpy as np
from tdmclient import ClientAsync



thymio_to_mms_conv = 0.43478260869565216

#Defining state space model x_k1 = A*x_k+B*u+v and y = C*x+w (discrete state space with disturbance and noise)