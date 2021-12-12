# Potential Field: https://medium.com/nerd-for-tech/local-path-planning-using-virtual-potential-field-in-python-ec0998f490af
# Bresenham algorithm: https://pypi.org/project/PyBresenham/

import numpy as np
import math
import pybresenham
import matplotlib.pyplot as plt

alpha = math.radians(20)
sin1a = math.sin(alpha)
sin2a = math.sin(2*alpha)
sin3a = math.sin(3*alpha)
cos1a = math.cos(alpha)
cos2a = math.cos(2*alpha)
cos3a = math.cos(3*alpha)

plot = False                     # Plot figure

thymio_r = 8                    # thymio radius
thymio_width = 12
scale = 1                       # sclaing factor to go from cm to mm
s = 3                           # distance to goal from where on the potentials have same amplitude
s_rep = 6
r = 0.5                         # Point radius
att = 50                        # attractive force of goal
rep = -25                       # repulsive force of obstacles
# Map size
size = 15
x = np.arange(-size,size,1)
y = np.arange(-size,size,1)
X, Y = np.meshgrid(x,y)
# Robot position. Get pose from localization
#posx = 0
#posy = 0
#pos = [posx, posy]
#r_theta = 0                      # robot heading angle

# Robot goal. Set goal so that pose is at zero --> goal = [goalx-posx, goaly-posy]
goal_abs = [25, 0]              #[cm]

step_size = 2                   #[cm]


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
    y = -0.00375*x + 18.63+1
    for idx in range(len(x)):
        if y[idx] >= 18.62:
            y[idx] = 0
    return y


# Rotation matrix
def rotate(xy, angle):
    new_xy = [math.cos(angle)*xy[0] - math.sin(angle)*xy[1], math.sin(angle)*xy[0] + math.cos(angle)*xy[1]]
    return new_xy


def PotField(prox, thymio_position, r_theta, path):

    posx = -8
    posy = 0

    goal_abs = [path.x, path.y]

    #Creating dx and dy array
    dx = np.zeros_like(X)
    dy = np.zeros_like(Y)


    #del prox[6]
    #del prox[5]
    prox = np.array(prox)
    proxCm = prox2cm(prox)
    proxScale = proxCm*scale
    proxScale.tolist()
    global x
    global y

    # goal in robot reference
    goal = np.array(rotate((goal_abs[0]-thymio_position.x, thymio_position.y-goal_abs[1]), r_theta))/10 # mm to cm

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
    if ObstScale[0] != 0:
        xyP = getXY(2, proxScale, 0)
        xyR = getXY(0, proxScale, 0)
        R1 = [round((ObstScale[0][0]+xyR[0])/2), round((ObstScale[0][1]+xyR[1])/2)]
        P1 = [round((ObstScale[0][0]+xyP[0])/2), round((ObstScale[0][1]+xyP[1])/2)]
        lR = list(pybresenham.line(ObstScale[0][0], ObstScale[0][1], R1[0], R1[1]))
        lP = list(pybresenham.line(ObstScale[0][0], ObstScale[0][1], P1[0], P1[1]))
        obst.extend(np.array(lR)/scale)
        obst.extend(np.array(lP)/scale)

    if ObstScale[1] != 0:
        xyP = getXY(1, proxScale, 1)
        xyS = getXY(3, proxScale, 1)
        P1 = [round((ObstScale[1][0]+xyP[0])/2), round((ObstScale[1][1]+xyP[1])/2)]
        S1 = [round((ObstScale[1][0]+xyS[0])/2), round((ObstScale[1][1]+xyS[1])/2)]
        lS = list(pybresenham.line(ObstScale[1][0], ObstScale[1][1], S1[0], S1[1]))
        lP = list(pybresenham.line(ObstScale[1][0], ObstScale[1][1], P1[0], P1[1]))
        obst.extend(np.array(lS)/scale)
        obst.extend(np.array(lP)/scale)

    if ObstScale[2] != 0:
        xyS1 = getXY(2, proxScale, 2)
        xyS2 = getXY(4, proxScale, 2)
        S1 = [round((ObstScale[2][0]+xyS1[0])/2), round((ObstScale[2][1]+xyS1[1])/2)]
        S2 = [round((ObstScale[2][0]+xyS2[0])/2), round((ObstScale[2][1]+xyS2[1])/2)]
        lS1 = list(pybresenham.line(ObstScale[2][0], ObstScale[2][1], S1[0], S1[1]))
        lS2 = list(pybresenham.line(ObstScale[2][0], ObstScale[2][1], S2[0], S2[1]))
        obst.extend(np.array(lS1)/scale)
        obst.extend(np.array(lS2)/scale)

    if ObstScale[3] != 0:
        xyP = getXY(3, proxScale, 3)
        xyS = getXY(5, proxScale, 3)
        S2 = [round((ObstScale[3][0]+xyS[0])/2), round((ObstScale[3][1]+xyS[1])/2)]
        P2 = [round((ObstScale[3][0]+xyP[0])/2), round((ObstScale[3][1]+xyP[1])/2)]
        lS = list(pybresenham.line(ObstScale[3][0], ObstScale[3][1], S2[0], S2[1]))
        lP = list(pybresenham.line(ObstScale[3][0], ObstScale[3][1], P2[0], P2[1]))
        obst.extend(np.array(lS)/scale)
        obst.extend(np.array(lP)/scale)

    if ObstScale[4] != 0:
        xyR = getXY(4, proxScale, 4)
        xyP = getXY(6, proxScale, 4)
        P2 = [round((ObstScale[4][0]+xyP[0])/2), round((ObstScale[4][1]+xyP[1])/2)]
        R2 = [round((ObstScale[4][0]+xyR[0])/2), round((ObstScale[4][1]+xyR[1])/2)]
        lR = list(pybresenham.line(ObstScale[4][0], ObstScale[4][1], R2[0], R2[1]))
        lP = list(pybresenham.line(ObstScale[4][0], ObstScale[4][1], P2[0], P2[1]))
        obst.extend(np.array(lR)/scale)
        obst.extend(np.array(lP)/scale)

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
            elif d>r+s_rep:
              dx[i][j] = dx[i][j] + 0
              dy[i][j] = dy[i][j] + 0
            else:
              dx[i][j] = dx[i][j] + rep*(s_rep+r-d)*np.cos(theta)
              dy[i][j] = dy[i][j] + rep*(s_rep+r-d)*np.sin(theta)


    cnt = 0
    dx_sum = 0
    dy_sum = 0
    # sum up every force vector between the thymio and the obstacles
    for yi in range(thymio_width):
        y_eval = size-int(thymio_width/2)+yi
        for o in range(len(obstacles)):
            if abs(obstacles[o][1])<thymio_width/2:
                for xi in range(int(obstacles[o][0])):
                    x_eval = size + xi
                    cnt = cnt + 1
                    dx_sum = dx_sum + dx[y_eval][x_eval]
                    dy_sum = dy_sum + dy[y_eval][x_eval]
    # take the mean of the force vectors and normalize
    if dx_sum == 0 and dy_sum != 0:
        dy_mean = dy_sum / cnt
        dx_mean = 0
        dy_norm = dy_mean/abs(dy_mean)
        dx_norm = 0
    elif dx_sum != 0 and dy_sum == 0:
        dy_mean = 0
        dx_mean = dx_sum / cnt
        dy_norm = 0
        dx_norm = dx_mean/abs(dx_mean)
    elif dx_sum == 0 and dy_sum == 0:
        # take direction at thymio pose
        if dy[size][size] != 0:
            dy_norm = dy[size][size]/abs(dy[size][size])
        else:
            dy_norm = 0
        if dx[size][size] != 0:
            dx_norm = dx[size][size]/abs(dx[size][size])
        else:
            dx_norm = 0
    else:
        dx_mean = dx_sum / cnt
        dy_mean = dy_sum / cnt
        if dy_mean > dx_mean:
            dy_norm = dy_mean/abs(dy_mean)
            dx_norm = dx_mean/abs(dy_mean)
        else:
            dx_norm = dx_mean/abs(dx_mean)
            dy_norm = dy_mean/abs(dx_mean)
    # scale up
    dx_next = dx_norm *step_size
    dy_next = dy_norm *step_size

    next = rotate((dx_next, dy_next), -r_theta)
    x_next = (next[0]+thymio_position.x/10)*10 # to have mm
    y_next = (-next[1]+thymio_position.y/10)*10 # to have mm
    #print('x_next', x_next, 'y_next', y_next, 'thym_next_x', next[0],'thym_next_y', -next[1])



    # ## compute next step
    # dx_ob = 0
    # dy_ob = 0
    # obstxy = [1,0]
    # # print(obstacles)
    # for p in range(5):
    #     if proxCm[p] != 0 and p != 2:
    #         closest_y = 100
    #         for o in range(len(obstacles)):
    #             if abs(obstacles[o][1]) < closest_y:
    #                 closest_y = abs(obstacles[o][1])
    #                 obstxy = obstacles[o]
    #
    # eval_x = int(obstxy[0]+size-int(obstxy[0]/2))
    # eval_y = int(obstxy[1]+size+int(obstxy[1]/2))
    #
    # dx_ob = dx[eval_y][eval_x]
    # dy_ob = dy[eval_y][eval_x]
    # if dy_ob > dx_ob:
    #     dy_norm = dy_ob/abs(dy_ob)
    #     dx_norm = dx_ob/abs(dy_ob)
    # else:
    #     dx_norm = dx_ob/abs(dx_ob)
    #     dy_norm = dy_ob/abs(dx_ob)
    # # scale up to step figsize
    # dx_cm = dx_norm*step_size
    # dy_cm = dy_norm*step_size
    # x_next = dx_cm + posx
    # y_next = dy_cm + posy
    # # break







    # posX = math.trunc(posx)
    # posY = math.trunc(posy)
    # # interpolation
    # nextX = (posx-posX)*(dx[posX-1+size][posY-1+size]-dx[posX+1-1+size][posY-1+size]) + dx[posX-1+size][posY-1+size]
    # nextY = (posy-posY)*(dy[posX-1+size][posY-1+size]-dy[posX-1+size][posY+1-1+size]) + dy[posX-1+size][posY-1+size]
    # print(nextY)

    if plot:
        fig, ax = plt.subplots(figsize = (10,10))
        ax.quiver(X, Y, dx, dy)
        ax.arrow(posx,posy,x_next/10,y_next/10, color='y')
        ax.add_patch(plt.Circle(goal, r, color='b'))
        for o in range(len(obstacles)):
            ax.add_patch(plt.Circle(obstacles[o], r, color='r'))
        ax.add_patch(plt.Circle([posx,posy], r, color='y'))
        plt.show()

    return [x_next,y_next]
