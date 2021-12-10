# Potential Field: https://medium.com/nerd-for-tech/local-path-planning-using-virtual-potential-field-in-python-ec0998f490af
# Bresenham algorithm: https://pypi.org/project/PyBresenham/

import numpy as np
import math
import pybresenham
import matplotlib.pyplot as plt
from local_motion_control import motion_control

alpha = math.radians(20)
sin1a = math.sin(alpha)
sin2a = math.sin(2*alpha)
sin3a = math.sin(3*alpha)
cos1a = math.cos(alpha)
cos2a = math.cos(2*alpha)
cos3a = math.cos(3*alpha)

plot = True                    # Plot figure

thymio_r = 8                    # thymio radius
scale = 10                       # sclaing factor to go from cm to mm
s = 3                           # distance to goal/obstacle from where on the potentials have same amplitude
r = 0.5                           # Point radius
att = 50                        # attractive force of goal
rep = -50                       # repulsive force of obstacles
# Map size
size = 15
x = np.arange(-size,size,1)
y = x
X, Y = np.meshgrid(x,y)
# Robot position. Get pose from localization
#posx = 0
#posy = 0
#pos = [posx, posy]

# Robot goal. Set goal so that pose is at zero --> goal = [goalx-posx, goaly-posy]
goal_abs = [25, 0]              #[mm]


# get x and y coordinate for a given prox value (in cm) and prox sensor with origin at the center of the robot
def getXY(sensor, proxCm, proxToEval=10):
    if proxToEval == 10:
        #default: proxToEval[0...4] = sensor[1...5]. Same sensor to eval than the one that gives the value
        proxToEval = (sensor-1)
    if sensor == 1:
        xy = [(thymio_r+proxCm[proxToEval])*cos2a, (thymio_r+proxCm[proxToEval])*sin2a]
    elif sensor == 2:
        xy = [(thymio_r+proxCm[proxToEval])*cos1a, (thymio_r+proxCm[proxToEval])*sin1a]
    elif sensor == 3:
        xy = [(thymio_r+proxCm[proxToEval]), 0]
    elif sensor == 4:
        xy = [(thymio_r+proxCm[proxToEval])*cos1a, -(thymio_r+proxCm[proxToEval])*sin1a]
    elif sensor == 5:
        xy = [(thymio_r+proxCm[proxToEval])*cos2a, -(thymio_r+proxCm[proxToEval])*sin2a]
    # Those are not real sensors but they help to compute the values for the obstacles at the two extrems
    elif sensor == 0:
        xy = [(thymio_r+proxCm[proxToEval])*cos3a, (thymio_r+proxCm[proxToEval])*sin3a]
    elif sensor == 6:
        xy = [(thymio_r+proxCm[proxToEval])*cos3a, -(thymio_r+proxCm[proxToEval])*sin3a]
    xy[0] = xy[0]-thymio_r
    return xy

# convert values of proximity sensor to cm
def prox2cm(x):
    y = -0.00375*x + 18.63
    for idx in range(len(x)):
        if y[idx] >= 18.62:
            y[idx] = 0
    return y


# Rotation matrix
def rotate(xy, angle):
    new_xy = (math.cos(angle)*xy[0] - math.sin(angle)*xy[1], math.sin(angle)*xy[0] + math.cos(angle)*xy[1])
    return new_xy


def PotField(node, variables, pos, theta, goal_abs):

    posx = pos.x
    posy = pos.y
    #Creating dx and dy array
    dx = np.zeros_like(X)
    dy = np.zeros_like(Y)

    prox = []
    for i in range(5):
        prox.append(node.v.prox.horizontal[i])
    print(prox)
    prox = np.array(prox)
    print(prox)
    proxCm = prox2cm(prox)
    proxScale = proxCm*scale
    proxScale.tolist()

    # goal in robot reference
    goal = rotate((goal_abs.x-pos.x, goal_abs.y-pos.y), -theta)

    # Obstacle distance detected by each sensor
    sens_obstacles = [0 for _ in range(len(proxCm))]     # distance given in cm
    for i in range(len(proxCm)):
        if proxCm[i] != 0:
            sens_obstacles[i] = getXY(i+1,proxCm)

    # inflate obstacles inbetween two sensors
    S1 = P1 = R1 = S2 = P2 = R2 = 0
    ObstScale = []
    for i in range(len(sens_obstacles)):
        array = np.array([sens_obstacles[i]])
        arr = array*scale
        ObstScale.extend(arr.tolist())
    obst = []
    if sens_obstacles[0] != 0:
        xyP = getXY(2, proxScale, 0)
        xyR = getXY(0, proxScale, 0)
        R1 = [round((ObstScale[0][0]+xyR[0])/2), round((ObstScale[0][1]+xyR[1])/2)]
        P1 = [round((ObstScale[0][0]+xyP[0])/2), round((ObstScale[0][1]+xyP[1])/2)]
        lR = list(pybresenham.line(ObstScale[0][0], ObstScale[0][1], R1[0], R1[1]))
        lP = list(pybresenham.line(ObstScale[0][0], ObstScale[0][1], P1[0], P1[1]))
        obst.extend(np.array(lR)/scale)
        obst.extend(np.array(lP)/scale)

    if sens_obstacles[1] != 0:
        xyP = getXY(1, proxScale, 1)
        xyS = getXY(3, proxScale, 1)
        P1 = [round((ObstScale[1][0]+xyP[0])/2), round((ObstScale[1][1]+xyP[1])/2)]
        S1 = [round((ObstScale[1][0]+xyS[0])/2), round((ObstScale[1][1]+xyS[1])/2)]
        lS = list(pybresenham.line(ObstScale[1][0], ObstScale[1][1], S1[0], S1[1]))
        lP = list(pybresenham.line(ObstScale[1][0], ObstScale[1][1], P1[0], P1[1]))
        obst.extend(np.array(lS)/scale)
        obst.extend(np.array(lP)/scale)

    if sens_obstacles[2] != 0:
        xyS1 = getXY(2, proxScale, 2)
        xyS2 = getXY(4, proxScale, 2)
        S1 = [round((ObstScale[2][0]+xyS1[0])/2), round((ObstScale[2][1]+xyS1[1])/2)]
        S2 = [round((ObstScale[2][0]+xyS2[0])/2), round((ObstScale[2][1]+xyS2[1])/2)]
        lS1 = list(pybresenham.line(ObstScale[2][0], ObstScale[2][1], S1[0], S1[1]))
        lS2 = list(pybresenham.line(ObstScale[2][0], ObstScale[2][1], S2[0], S2[1]))
        obst.extend(np.array(lS1)/scale)
        obst.extend(np.array(lS2)/scale)

    if sens_obstacles[3] != 0:
        xyP = getXY(3, proxScale, 3)
        xyS = getXY(5, proxScale, 3)
        S2 = [round((ObstScale[3][0]+xyS[0])/2), round((ObstScale[3][1]+xyS[1])/2)]
        P2 = [round((ObstScale[3][0]+xyP[0])/2), round((ObstScale[3][1]+xyP[1])/2)]
        lS = list(pybresenham.line(ObstScale[3][0], ObstScale[3][1], S2[0], S2[1]))
        lP = list(pybresenham.line(ObstScale[3][0], ObstScale[3][1], P2[0], P2[1]))
        obst.extend(np.array(lS)/scale)
        obst.extend(np.array(lP)/scale)

    if sens_obstacles[4] != 0:
        xyR = getXY(4, proxScale, 4)
        xyP = getXY(6, proxScale, 4)
        P2 = [round((ObstScale[4][0]+xyP[0])/2), round((ObstScale[4][1]+xyP[1])/2)]
        R2 = [round((ObstScale[4][0]+xyR[0])/2), round((ObstScale[4][1]+xyR[1])/2)]
        lR = list(pybresenham.line(ObstScale[4][0], ObstScale[4][1], R2[0], R2[1]))
        lP = list(pybresenham.line(ObstScale[4][0], ObstScale[4][1], P2[0], P2[1]))
        obst.extend(np.array(lR)/scale)
        obst.extend(np.array(lP)/scale)

    obstacles = []
    obstacles = np.array(obst)

    # Potential field of goal
    for i in range(len(x)):
        for j  in range(len(y)):
            # Calculation the distance to the goal
            d = np.sqrt((goal[0]-X[i][j])**2 + (goal[1]-Y[i][j])**2)
            #Calculation angle
            theta = np.arctan2(goal[1]-Y[i][j], goal[0]-X[i][j])
            # x and y componants of the potential at each point
            if d < r:
                dx[i][j] = 0
                dy[i][j] = 0
            elif d > r + s:
                dx[i][j] = att*s*np.cos(theta)
                dy[i][j] = att*s*np.sin(theta)
            else:
                dx[i][j] = att*(d-r)*np.cos(theta)
                dy[i][j] = att*(d-r)*np.sin(theta)


    # Potential field of obstacles
    for o in range(len(obstacles)):
        for i in range(len(x)):
          for j in range(len(y)):
            # Calculation the distance to the obstacle
            #d = np.sqrt((obstacles[0]-X[i][j])**2 + (obstacles[1]-Y[i][j])**2)
            d = np.sqrt((obstacles[o][0]-X[i][j])**2 + (obstacles[o][1]-Y[i][j])**2)
            #Calculation angle
            #theta = np.arctan2(obstacles[1]-Y[i][j], obstacles[0]-X[i][j])
            theta = np.arctan2(obstacles[o][1]-Y[i][j], obstacles[o][0]-X[i][j])
            # x and y componants of the potential at each point
            if d<r:
              dx[i][j] = dx[i][j] + np.sign(np.cos(theta))
              dy[i][j] = dy[i][j] + np.sign(np.cos(theta))
            elif d>r+s:
              dx[i][j] = dx[i][j] + 0
              dy[i][j] = dy[i][j] + 0
            else:
              dx[i][j] = dx[i][j] + rep*(s+r-d)*np.cos(theta)
              dy[i][j] = dy[i][j] + rep*(s+r-d)*np.sin(theta)

    ## compute next step
    posX = math.trunc(posx)
    posY = math.trunc(posy)
    # interpolation
    nextX = (posx-posX)*(dx[posX-1+size][posY-1+size]-dx[posX+1-1+size][posY-1+size]) + dx[posX-1+size][posY-1+size]
    nextY = (posy-posY)*(dy[posX-1+size][posY-1+size]-dy[posX-1+size][posY+1-1+size]) + dy[posX-1+size][posY-1+size]
    print("NextY", nextY)
    motion_control(node, variables, [posX,posY], theta, [nextX,nextY])

    #if plot:
    #    fig, ax = plt.subplots(figsize = (10,10))
    #    ax.quiver(X, Y, dx, dy)
    #    ax.add_patch(plt.Circle(goal, r, color='b'))
    #    for o in range(len(obstacles)):
    #        ax.add_patch(plt.Circle(obstacles[o], r, color='r'))
    #    ax.add_patch(plt.Circle(pos, r, color='y'))
    #    plt.show()
