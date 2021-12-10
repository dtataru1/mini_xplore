from tdmclient import ClientAsync
from tdmclient.atranspiler import ATranspiler
import sys


def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def motion_control(node, variables):
  node.send_set_variables(motors(0, 0))



with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(motion_control)

            # Will sleep forever:
            await client.sleep()
    client.run_async_program(prog)
