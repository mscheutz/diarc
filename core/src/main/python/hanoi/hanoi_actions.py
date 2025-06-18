import sys
import logging

logging.basicConfig(stream=sys.stdout, level=logging.INFO)

from . import kortex_controller
from pytrade.wrapper import TRADEWrapper

wrapper = TRADEWrapper()


def get_all_pos():
    tokens = wrapper.call_trade("getTokens", [])
    token_map = {}
    for token in tokens:
        loc = token.getLocation()
        token_map[token.getTypeId()] = [loc.x, loc.y, loc.z]

    return token_map


def move_arm(action):
    logging.info(f"Moving arm {action}")
    # kortex_controller.go_to(action)

def check_gripping():
    # Todo: Check via diarc or... Direct through API?
    return True, True

def main():
    relative_move = [0.2, 0, 0.2]
    kortex_controller.go_to_relative(relative_move)  # Go to home pos


if __name__ == "__main__":
    main()
