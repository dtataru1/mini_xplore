import math
import random
from constants import *

global vl, vr
vl = 0
vr = 0

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

# Control of the motor to get to the next position given
def motion_control(node, pos, phi, next_step, path, goal_threshold, motor_left_speed, motor_right_speed):

    if math.sqrt((path[0].y-pos.y)**2+(path[0].x-pos.x)**2) > goal_threshold:  # Not at goal yet, keep moving in the desired position
        phi_d = math.atan2(next_step[1]-pos.y,next_step[0]-pos.x)
        w = K*(phi_d-phi)

        motor_left_speed = 100*(v0/R + w*L/(2*R))
        motor_right_speed = 100*(v0/R - w*L/(2*R))
        node.send_set_variables(motors(int(motor_left_speed), int(motor_right_speed)))

    elif goal_threshold == FINAL_GOAL:  # Arrived at final goal -> stops motors
        motor_left_speed = 0
        motor_right_speed = 0
        node.send_set_variables(motors(motor_left_speed, motor_right_speed))
        path.pop(0)


    elif goal_threshold == STEP_GOAL:   # Updates goal to next one
        path.pop(0)

    return [motor_left_speed, motor_right_speed]
