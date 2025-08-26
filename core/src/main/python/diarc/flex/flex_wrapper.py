import logging
import argparse

from diarc.flex.env.PrismaticEnv import PrismaticEnv
from diarc.flex.env.RevoluteEnv import RevoluteEnv
from policies import door_open, grasp_handle, push_button # from flex
from sklearn.utils import Bunch

from diarc.flex.DIARC_env import DIARCEnv
from diarc.pytrade.java_util import to_java_object, to_java_object_from_class
from diarc.pytrade.wrapper import TRADEWrapper

from jpype import JImplements, JOverride, JClass, JString

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import RLInterface
from edu.tufts.hrilab.action.justification import ConditionJustification, Justification
from edu.tufts.hrilab.fol import Factory, Symbol, Predicate

logger = logging.getLogger(__name__)


@JImplements(RLInterface)
class FlexWrapper:
    def __init__(self, trade_wrapper: TRADEWrapper):
        self.trade = trade_wrapper
        self.analysis_env = AnalysisEnv(trade_wrapper)
        self.prismatic_env = PrismaticEnv(trade_wrapper)
        self.revolute_env = RevoluteEnv(trade_wrapper)
        self.joint_type = 'unknown'
        self.joint_params = None

        # pass in config here, or should those be stored in the env?
        config = Bunch()
        config.max_steps = 50
        config.action_scale = 0.02
        config.success_distance = 0.1
        config.success_angle = 60.0
        self.flex = flex.FlexPolicies(config)

    @JOverride
    def callPolicy(self, policy_name: str):
        try:
            if policy_name == 'analyze_joint':
                self.joint_type, self.joint_params = self.flex.analyze_door_joint(self.analysis_env)
            elif policy_name == 'open_door':
                # execute open door policy -- only if analyze_door_joint has been called
                if self.joint_type == 'prismatic':
                    self.prismatic_env.joint_params = self.joint_params
                    (success, msg) = self.flex.execute_policy(self.prismatic_env)
                    self._handle_return(success)
                if self.joint_type == 'revolute':
                    self.revolute_env.joint_params = self.joint_params
                    (success, msg) = self.flex.execute_policy(self.revolute_env)
                    self._handle_return(success)
                else:
                   logger.error(f"Invalid joint_type: {self.joint_type}. Did you call the analyze_joint policy first?")
            else:
               logger.error(f"Invalid policy name: {policy_name}")
        except Exception as e:
            logging.error(f"Caught exception calling flex policy: {policy_name}. Error: {e}")

        return self._handle_return(False)

    @JOverride
    def learnPolicy(self, action: str):
        pass

    @JOverride
    def updatePolicy(self, failed_operator: str):
        pass

    def _handle_return(self, value: bool, reason: Predicate = None) -> Justification:
        if reason is None:
            return ConditionJustification(value)
        return ConditionJustification(value, reason)

if __name__ == '__main__':
    trade_wrapper = TRADEWrapper()

    parser = argparse.ArgumentParser(description='Flex TRADE Wrapper')
    parser.add_argument('--groups', nargs='*')
    args = parser.parse_args()

    flex_wrapper = FlexWrapper(trade_wrapper)
    TRADE.registerAllServices(flex_wrapper, args.groups if args.groups else "")
