from tdmclient import ClientAsync
from tdmclient.atranspiler import ATranspiler
import math
import random

global K, v0, R, L, T, x1, x2, y1, y2, phi
K = 0.2
v0 = 1.0
R = 2.0 # wheel radius
L = 9.0 # axle length
T = 0.1 # time constant between measures: motor control speed = 100Hz (@datasheet), taken at 10 Hz here

x1 = 0
y1 = 0
phi = math.pi/4

#x2 = 20*random.random()
#y2 = 20*random.random()
#phi = 2*math.pi.*random.random() - math.pi

x2 = -500
y2 = -500
#phi = 2*math.pi*random.random() - math.pi
#print(phi)

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def on_variables_changed(node, variables):
    global K, v0, R, L, T, x1, x2, y1, y2, phi
    try:
        #print("x1",x1,"y1",y1,"x2",x2,"y2",y2)
        #print("sqrt",math.sqrt((y2-y1)**2+(x2-x1)**2))
        if math.sqrt((y2-y1)**2+(x2-x1)**2) > 10:
            phi_d = math.atan2(y2-y1,x2-x1)
            print(" ")
            print("phi_d", phi_d, "phi", phi)
            w = K*(phi_d-phi)

            print("w",w)
            vl = 100*(v0/R - w*L/(2*R))
            vr = 100*(v0/R + w*L/(2*R))
            print("vl",vl,"vr",vr)
            x1 = x1 + (vl+vr)/2*math.cos(phi)/25 # /25 because wheels take value up to 500 which corresponds to 20 cm/s
            y1 = y1 + (vl+vr)/2*math.sin(phi)/25
            print("x1",x1,"y1",y1)
            phi = phi + w*T
            node.send_set_variables(motors(int(vl), int(vr)))
        else:
            node.send_set_variables(motors(0, 0))

    except KeyError:
        pass  # prox.horizontal not found

with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(on_variables_changed)
            # Will sleep forever:
            await client.sleep()
    client.run_async_program(prog)
