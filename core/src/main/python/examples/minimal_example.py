import time
import sys

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
    wrapper = TRADEWrapper()
    dummyObject = dummyWrapper()
    TRADE.registerAllServices(dummyObject, "")
    time.sleep(1)
    print(TRADE.getAvailableServices())
    wrapper.call_trade("undock", )

