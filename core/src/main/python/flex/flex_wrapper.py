import logging
import os
import time
import typing
import policies # from flex

from jpype import JImplements, JOverride, JClass, JString
from pytrade.wrapper import TRADEWrapper

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import FlexPythonInterface
from edu.tufts.hrilab.action.justification import ConditionJustification
from edu.tufts.hrilab.fol import Factory, Symbol, Predicate
from edu.tufts.hrilab.fol import Factory, Symbol, Predicate

logger = logging.getLogger(__name__)


@JImplements([FlexPythonInterface])
class FlexWrapper:
    def __init__(self, trade_wrapper: TRADEWrapper):
        self.trade = trade_wrapper

    @JOverride
    def door_open_primitive(self, x_pix, y_pix):

        # TODO: call flex policy
        success = policies.door_open(x_pix, y_pix)

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
