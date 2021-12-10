import numpy as np
import cv2
import matplotlib.pyplot as plt
from constants import *
from tdmclient import ClientAsync

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def main(node, variables):
    try:

        front_tof = []
        for i in range(5):
            front_tof.append(node.v.prox.horizontal[i])
        print(front_tof)
        #print(node.v.prox.horizontal[1])

    except KeyError:
        pass  # prox.horizontal not found


with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            #main(node,node.v)
            node.add_variables_changed_listener(main)
            # Will sleep forever:
            await client.sleep()
    client.run_async_program(prog)
