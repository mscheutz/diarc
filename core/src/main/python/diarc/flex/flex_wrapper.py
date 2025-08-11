import logging
import os
import time
import typing
from policies import door_open # from flex

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
    def __init__(self, trade_wrapper: TRADEWrapper):
        self.trade = trade_wrapper

    @JOverride
    def open_door_primitive(self, x_pix, y_pix):
        # call flex policy
        logging.info(f"calling flex door_open policy with pixel coordinates: {x_pix}, {y_pix}")
        success = door_open.door_open_pixel_location(x_pix, y_pix)

        return self._handle_return(success)

    @JOverride
    def grasp_object_primitive(self, x_pix, y_pix):
        # call flex policy
        logging.info(f"calling flex grasp_handle policy with pixel coordinates: {x_pix}, {y_pix}")
        success = door_open.grasp_handle_pixel_location(x_pix, y_pix)

        return self._handle_return(success)

    @JOverride
    def push_button_primitive(self, x_pix, y_pix):
        # call flex policy
        logging.info(f"calling flex push_button policy with pixel coordinates: {x_pix}, {y_pix}")
        success = door_open.push_button_pixel_location(x_pix, y_pix)

        return self._handle_return(success)

    def _handle_return(self, value: bool, reason: Predicate = None):
        if reason is None:
            return ConditionJustification(value)
        return ConditionJustification(value, reason)

# This main is largely used for testing behaviors
if __name__ == '__main__':
    trade_wrapper = TRADEWrapper()

    nadia_wrapper = FlexWrapper(trade_wrapper)
    TRADE.registerAllServices(nadia_wrapper, "")
