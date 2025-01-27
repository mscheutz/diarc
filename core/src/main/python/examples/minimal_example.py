import time
import sys
import logging
from pytrade.wrapper import TRADEWrapper

from ai.thinkingrobots.trade import TRADE
from jpype import JImplements, JOverride
from edu.tufts.hrilab.interfaces import DockingInterface


@JImplements(DockingInterface)
class dummyWrapper:
    @JOverride
    def dock(self, dockId):
        pass

    @JOverride
    def undock(self):
        print("Undocking")
        pass


if __name__ == '__main__':
    # Todo: Print doesn't working, need to use logging. Figure out why.
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    logging.info("This will show up in Java output")

    wrapper = TRADEWrapper()
    dummyObject = dummyWrapper()
    TRADE.registerAllServices(dummyObject, "")
    time.sleep(1)
    wrapper.call_trade("undock")