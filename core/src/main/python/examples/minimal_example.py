import time
import sys
import logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)

from pytrade.wrapper import TRADEWrapper
from ai.thinkingrobots.trade import TRADE
from jpype import JImplements, JOverride
from edu.tufts.hrilab.interfaces import DockingInterface

@JImplements(DockingInterface)
class DockingComponent:
    @JOverride
    def dock(self, dockId):
        logging.info(f"Docking: {dockId}")

    @JOverride
    def undock(self):
        logging.info("Undocking")


if __name__ == '__main__':

    wrapper = TRADEWrapper()
    docking_component = DockingComponent()
    TRADE.registerAllServices(docking_component, "")
    time.sleep(1)

    while True:
        wrapper.call_trade("undock")
        time.sleep(5)