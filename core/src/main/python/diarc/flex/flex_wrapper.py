import logging
import argparse
import os

# Import from FLEX repo
from flex.executor import FlexExecutor
from flex.config import FlexConfig

from diarc.pytrade.java_util import to_java_object, to_java_object_from_class
from diarc.pytrade.wrapper import TRADEWrapper
from jpype import JImplements, JOverride, JClass, JString
from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.rl import RLInterface
from edu.tufts.hrilab.action.justification import ConditionJustification, Justification
from edu.tufts.hrilab.fol import Factory, Symbol, Predicate

from diarc.flex.env.AnalysisEnv import AnalysisEnv
from diarc.flex.env.PrismaticEnv import PrismaticEnv
from diarc.flex.env.RevoluteEnv import RevoluteEnv

logger = logging.getLogger(__name__)


@JImplements(RLInterface)
class FlexWrapper:
    def __init__(self, trade_wrapper: TRADEWrapper):
        self.trade = trade_wrapper

        # Create gym environments
        self.analysis_env = AnalysisEnv(trade_wrapper)
        self.prismatic_env = PrismaticEnv(trade_wrapper)
        self.revolute_env = RevoluteEnv(trade_wrapper)

        # State tracking
        self.joint_type = None
        self.joint_params = None

        home_dir = os.path.expanduser("~")
        prismatic_model_dir = os.path.join(home_dir, ".diarc", "flex", "models", "prismatic")
        revolute_model_dir = os.path.join(home_dir, ".diarc", "flex", "models", "revolute")

        # Create FlexConfig
        config = FlexConfig(
            max_steps=50,
            action_scale=0.02,
            success_distance=0.1,
            success_angle=60.0,
            movement_distance=0.05,
            prismatic_policy_path=prismatic_model_dir,
            revolute_policy_path=revolute_model_dir
        )

        # Create FlexExecutor
        self.flex_executor = FlexExecutor(config)

        # PRE-LOAD policies to avoid lazy loading during shutdown
        try:
            logger.info("Pre-loading policies...")
            self.flex_executor._get_policy('prismatic')
            self.flex_executor._get_policy('revolute')
            logger.info("Policies pre-loaded successfully")
        except Exception as e:
            logger.warning(f"Could not pre-load policies: {e}")

    @JOverride
    def callPolicy(self, policy_name: str):
        logger.info(f"FLEX call policy: {policy_name}")

        try:
            if policy_name == 'analyze_joint':
                return self._analyze_joint()

            elif policy_name == 'open_door':
                return self._open_door()

            else:
                logger.error(f"Invalid policy name: {policy_name}")
                return self._handle_return(False)

        except Exception as e:
            logger.error(f"Caught exception calling flex policy: {policy_name}. Error: {e}", exc_info=True)
            return self._handle_return(False)

    def _analyze_joint(self):
        """Phase 1: Joint analysis via wiggle movements."""
        logger.info("Starting joint analysis...")

        # Delegate to FlexExecutor
        self.joint_type, self.joint_params = self.flex_executor.analyze_joint(self.analysis_env)
        
        logger.info(f"Analysis complete: {self.joint_type}")
        logger.info(f"Joint params: {self.joint_params}")

        return self._handle_return(self.joint_type is not None)

    def _open_door(self):
        """Phase 2: Policy-based door opening."""
        if self.joint_type is None or self.joint_params is None:
            logger.error("Must call analyze_joint before open_door")
            return self._handle_return(False)

        logger.info(f"Executing door opening with {self.joint_type} policy...")

        # Select appropriate environment
        if self.joint_type == 'prismatic':
            env = self.prismatic_env
        elif self.joint_type == 'revolute':
            env = self.revolute_env
        else:
            logger.error(f"Invalid joint_type: {self.joint_type}")
            return self._handle_return(False)

        # Execute policy via FlexExecutor
        success, msg = self.flex_executor.execute_policy(
            env, self.joint_type, self.joint_params
        )

        logger.info(f"Execution result: {msg}")
        return self._handle_return(success)

    @JOverride
    def cancelPolicy(self):
        self.flex_executor.cancel_policy()
        return self._handle_return(True)

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
