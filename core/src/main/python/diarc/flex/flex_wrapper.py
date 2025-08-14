import logging
import os
import time
import argparse
import typing
import yaml
from policies import door_open, grasp_handle, push_button # from flex
from sklearn.utils import Bunch

from diarc.pytrade.java_util import to_java_object, to_java_object_from_class
from diarc.pytrade.wrapper import TRADEWrapper

from jpype import JImplements, JOverride, JClass, JString

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import FlexPythonInterface
from edu.tufts.hrilab.action.justification import ConditionJustification
from edu.tufts.hrilab.fol import Factory, Symbol, Predicate

logger = logging.getLogger(__name__)


@JImplements(FlexPythonInterface)
class FlexWrapper:
    def __init__(self, trade_wrapper: TRADEWrapper, username, password, hostname):
        self.trade = trade_wrapper
        self.username = username
        self.password = password
        self.hostname = hostname


    @JOverride
    def open_door_primitive(self, x_pix, y_pix):
        config = Bunch()
        config.username = self.username
        config.password = self.password
        config.hostname = self.hostname
        config.image_source = "hand_color_image"
        config.max_steps = 50
        config.action_scale = 0.02
        config.success_distance = 0.1
        config.success_angle = 60.0

        # call flex policy
        logging.info(f"calling flex door_open policy with pixel coordinates: {x_pix}, {y_pix}, {config}")
        success = door_open.door_open_pixel_location(x_pix, y_pix, config)

        return self._handle_return(success)

    @JOverride
    def grasp_object_primitive(self, x_pix, y_pix):
        config = Bunch()
        config.username = self.username
        config.password = self.password
        config.hostname = self.hostname
        config.image_source = "hand_color_image"

        # call flex policy
        logging.info(f"calling flex grasp_handle policy with pixel coordinates: {x_pix}, {y_pix}, {config}")
        success = grasp_handle.grasp_handle_pixel_location(x_pix, y_pix, config)

        return self._handle_return(success)

    @JOverride
    def push_button_primitive(self, x_pix, y_pix):
        config = Bunch()
        config.username = self.username
        config.password = self.password
        config.hostname = self.hostname
        config.image_source = "hand_color_image"
        config.approach_distance = 1
        config.press_force_percentage = 0.05
        config.press_duration = 1.0
        config.walk_timeout = 15.0

        # call flex policy
        logging.info(f"calling flex push_button policy with pixel coordinates: {x_pix}, {y_pix}, {config}")
        try:
            success = push_button.push_button_pixel_location(x_pix, y_pix, config)
        except Exception as e:
            logging.error(f"Caught a general Java exception: {e}")
            return False

        return self._handle_return(success)

    def _handle_return(self, value: bool, reason: Predicate = None):
        if reason is None:
            return ConditionJustification(value)
        return ConditionJustification(value, reason)

# This main is largely used for testing behaviors
if __name__ == '__main__':
    trade_wrapper = TRADEWrapper()

    parser = argparse.ArgumentParser(description='Flex TRADE Wrapper')
    parser.add_argument('--spot-config', required=True, help='Path to Spot yaml config file')
    parser.add_argument('--groups', nargs='*')
    args = parser.parse_args()

    # load spot config yaml and get relevant bits
    with open(args.spot_config, 'r') as file:
        config = yaml.safe_load(file)
        config = config['/**']['ros__parameters']

    username = config["username"]
    password = config["password"]
    hostname = config["hostname"]
    logging.info(f"Parsed config: username={username}, password={password}, hostname{hostname}")

    flex_wrapper = FlexWrapper(trade_wrapper, username, password, hostname)
    TRADE.registerAllServices(flex_wrapper, args.groups if args.groups else "")
