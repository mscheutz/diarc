import time
import sys
import logging

logging.basicConfig(stream=sys.stdout, level=logging.INFO)

from pytrade.wrapper import TRADEWrapper
from jpype import JClass

wrapper = TRADEWrapper()
from javax.vecmath import Point3d
from javax.vecmath import Quat4d


def get_all_pos():
    tokens = wrapper.call_trade("getTokens", [])
    token_map = {}
    for token in tokens:
        loc = token.getLocation()
        token_map[token.getTypeId()] = [loc.x, loc.y, loc.z]

    return token_map


def move_arm(action):
    pt = Point3d(*action[:3])
    qt = Quat4d(*action[3:])
    ret = wrapper.call_trade("moveTo", [], "group", pt, qt)  # todo: make safe, also use ret to check success
    return ret.getValue()


while True:
    time.sleep(1) # We'll remove this once we've got everything working
