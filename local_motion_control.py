import math
import random
from constants import *

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def motion_control(node, pos, phi, next_step, path, goal_threshold):

    if math.sqrt((path[0].y-pos.y)**2+(path[0].x-pos.x)**2) > goal_threshold:
        phi_d = math.atan2(next_step[1]-pos.y,next_step[0]-pos.x)
        #print("phi_d ", phi_d, "phi ", phi, "position ", pos, "destination ", goal_abs)
        w = K*(phi_d-phi)

        vl = 100*(v0/R + w*L/(2*R))
        vr = 100*(v0/R - w*L/(2*R))
        node.send_set_variables(motors(int(vl), int(vr)))

    elif goal_threshold == FINAL_GOAL:
        node.send_set_variables(motors(0, 0))
        path.pop(0)

    elif goal_threshold == STEP_GOAL:
        path.pop(0)     # Updates goal
