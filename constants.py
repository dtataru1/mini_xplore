import pyvisgraph as vg
import numpy as np
import math

# Thymio values
PC_PF_DIST = 62.5               #mm
PC_SUPPORT_DIST = 27.5          #mm
TRIANGLE_SUPPORT = 45           #mm
TRIANGLE_EDGE = 93              #mm
THYMIO_TRIANGLE_TRESHOLD = (TRIANGLE_SUPPORT+TRIANGLE_EDGE)/2 #mm
R = 15                          # wheel radius mm
L = 80                          # axle length in mm
v0 = 15                         # basic wheel speed

# Local motion
K = 0.2                         # gain
FINAL_GOAL = 80
STEP_GOAL = 60

# Computer Vision Constants
THRESHOLD_AREA = 1000
EPSILON_THYMIO = 0.06
EPSILON_OBSTACLES = 0.03
EPSILON_GOAL = 0.03
OBST_INCREASE = 80.0

# Local Planner
alpha = math.radians(20)
sin1a = math.sin(alpha)
sin2a = math.sin(2*alpha)
sin3a = math.sin(3*alpha)
cos1a = math.cos(alpha)
cos2a = math.cos(2*alpha)
cos3a = math.cos(3*alpha)
thymio_r = 8                    # thymio radius
thymio_width = 16
scale = 1                       # sclaing factor to go from cm to mm
s = 3                           # distance to goal from where on the potentials have same amplitude
s_rep = 6
r = 0.5                         # Point radius
att = 50                        # attractive force of goal
rep = -35                       # repulsive force of obstacles
size = 15                       # Map size
x = np.arange(-size,size,1)
y = np.arange(-size,size,1)
X, Y = np.meshgrid(x,y)
step_size = 2                   #[cm]

# Map and Pixel sizes
CAM_Y_PIXEL_SIZE = 1080         #pixels
CAM_X_PIXEL_SIZE = 1920         #pixels
MAP_Y_MM_SIZE = 790             #mm
MAP_X_MM_SIZE = 1430            #mm
CAM_2_MAP = MAP_X_MM_SIZE/CAM_X_PIXEL_SIZE

# Kalman Constants
Ts = 0.30
speed_conv_factor = 0.20        #Speed from thymio speed to mm/s (to confirm)
rot_conv_factor = -0.0031
P_est_prev = 10 * np.identity(6)#Initial P_est
x_est_prev = np.array([[0], [0], [0], [0], [0], [0]]) #Initial position
x_est_range = x_est_prev        #Matrix storing all values from the beginning
