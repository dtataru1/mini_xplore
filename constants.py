import pyvisgraph as vg
import numpy as np

# Thymio values
PC_PF_DIST = 62.5 #mm
PC_SUPPORT_DIST = 27.5 #mm
TRIANGLE_SUPPORT = 45 #mm
TRIANGLE_EDGE = 93 #mm
THYMIO_TRIANGLE_TRESHOLD = (TRIANGLE_SUPPORT+TRIANGLE_EDGE)/2 #mm
R = 15 # wheel radius mm
L = 80 # axle length in mm
v0 = 10 # basic wheel speed

# Local motion
K = 0.2 # gain
FINAL_GOAL = 20
STEP_GOAL = 20
#T = 0.1 # time constant between measures: motor control speed = 100Hz (@datasheet), taken at 10 Hz here

# Computer Vision Constants
THRESHOLD_AREA = 1000
EPSILON_THYMIO = 0.06
EPSILON_OBSTACLES = 0.03
EPSILON_GOAL = 0.03

# Map and Pixel sizes
CAM_Y_PIXEL_SIZE = 1080 #pixels
CAM_X_PIXEL_SIZE = 1920  #pixels
MAP_Y_MM_SIZE = 790 #mm
MAP_X_MM_SIZE = 1430 #mm
CAM_2_MAP = MAP_X_MM_SIZE/CAM_X_PIXEL_SIZE

# Kalman Constants

Ts = 0.25
speed_conv_factor = 0.36 #Speed from thymio speed to mm/s (to confirm)

rot_conv_factor = 0.0066

P_est_prev = 10 * np.identity(6) #Initial P_est
x_est_prev = np.array([[0], [0], [0], [0], [0], [0]]) #Initial position
x_est_range = x_est_prev #Matrix storing all values from the beginning
