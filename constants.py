import pyvisgraph as vg

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
FINAL_GOAL = 120
STEP_GOAL = 40
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

# Global planner Constants

#START_POSITION = vg.Point(5.0,5.5) ## Setup HERE
#END_POSITION = vg.Point(750.0,450.0)  ## Setup HERE
#THYMIO_POSITION = vg.Point(50.0,55.5)  ## Given by Daniel
