import os
import time
import sys
import logging

import numpy as np
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_controller import *

logging.basicConfig(stream=sys.stdout, level=logging.INFO)

# from pytrade.wrapper import TRADEWrapper
from jpype import JClass
import kortex_controller


# wrapper = TRADEWrapper()
# from javax.vecmath import Point3d
# from javax.vecmath import Quat4d


# def get_all_pos():
#     tokens = wrapper.call_trade("getTokens", [])
#     token_map = {}
#     for token in tokens:
#         loc = token.getLocation()
#         token_map[token.getTypeId()] = [loc.x, loc.y, loc.z]
#
#     return token_map
#
#
# def move_arm(action):
#     pt = Point3d(*action[:3])
#     qt = Quat4d(*action[3:])
#     ret = wrapper.call_trade("moveTo", [], "group", pt, qt)  # todo: make safe, also use ret to check success
#     return ret.getValue()



def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        init_pos = [0.7, 0, 0.5]
        cartesian_movement(base, base_cyclic, init_pos)  # Go to home pos


if __name__ == "__main__":
    main()
