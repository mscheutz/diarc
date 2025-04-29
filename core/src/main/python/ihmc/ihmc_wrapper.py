import logging
import time
import sys
import threading

import rclpy
from pytrade.java_util import to_java_object, to_java_object_from_class
from pytrade.wrapper import TRADEWrapper

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import IHMCInterface
from edu.tufts.hrilab.fol import Factory, Symbol, Predicate
from edu.tufts.hrilab.action.justification import ConditionJustification, Justification
from edu.tufts.hrilab.consultant.pose import PoseConsultant, PoseReference
from jpype import JImplements, JOverride, JClass, JString
from geometry_msgs.msg import Point, Quaternion, Pose
from ihmc_common_msgs.msg import PoseListMessage
from behavior_msgs.msg import AI2RCommandMessage, AI2RStatusMessage
from behavior_msgs.msg import ContinuousWalkingCommandMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


@JImplements(IHMCInterface)
class NadiaWrapper:
    def __init__(self, trade_wrapper: TRADEWrapper):
        self._executing = False
        self._behavior_status = ""
        self.trade_wrapper = trade_wrapper
        self.node = rclpy.create_node('diarc_node')
        self._qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._walk_timer = None
        self._walk_publisher = self.node.create_publisher(ContinuousWalkingCommandMessage,
                                                          '/ihmc/continuous_walking/command',
                                                          qos_profile=self._qos_profile)
        self._command_publisher = self.node.create_publisher(AI2RCommandMessage,
                                                             '/ihmc/behavior_tree/ai2r_command',
                                                             qos_profile=self._qos_profile)
        self._behavior_subscriber = self.node.create_subscription(AI2RStatusMessage,
                                                                  '/ihmc/behavior_tree/ai2r_status',
                                                                  self._behavior_callback,
                                                                  self._qos_profile)

        ### Consultant
        self.kb_name = to_java_object("location")
        self.consultant = PoseConsultant(JClass(PoseReference), self.kb_name,
                                         to_java_object_from_class([], "java.util.ArrayList"))

    def _behavior_callback(self, msg):
        for obj in msg.objects:
            name = obj.object_name
            position = obj.object_pose_in_world.position
            orientation = obj.object_pose_in_world.orientation
            # todo: update pose in consultant
        if self._executing:
            if msg.completed_behavior != '':
                self._behavior_status = "success"
            elif msg.failed_behavior != '':
                self._behavior_status = "failure"
            else:
                self._behavior_status = "progress"

    @JOverride
    def walk(self):
        logging.info("[walk]")
        command = ContinuousWalkingCommandMessage()
        command.publish_to_controller = True
        command.enable_continuous_walking = True
        self._walk_publisher.publish(command)

    @JOverride
    def push_door_primitive(self) -> Justification:
        logging.info("[push_door]")
        command = AI2RCommandMessage()
        command.behavior_to_execute = "PUSH DOOR"
        self._command_publisher.publish(command)
        self._executing = True

        while self._behavior_status == "progress" or self._behavior_status == "":
            logging.info("in progress...")
            time.sleep(1)

        if self._behavior_status == "success":
            ret = self._handle_return(True)
        elif self._behavior_status == "failure":
            ret = self._handle_return(False)
        else:
            ret = self._handle_return(False)

        self._behavior_status = ""
        self._executing = False
        return ret

    @JOverride
    def pull_door_primitive(self) -> Justification:
        logging.info("[pull_door]")
        command = AI2RCommandMessage()
        command.behavior_to_execute = "PULL DOOR"
        self._command_publisher.publish(command)
        self._executing = True
        logging.info(self._behavior_status)
        while self._behavior_status == "progress" or self._behavior_status == "":
            logging.info("in progress...")
            time.sleep(1)

        if self._behavior_status == "success":
            ret = self._handle_return(True)
        elif self._behavior_status == "failure":
            ret = self._handle_return(False)
        else:
            ret = self._handle_return(False)

        self._behavior_status = ""
        self._executing = False
        return ret

    # Todo
    def _get_pose_from_symbol(self, symbol):
        obj = self.trade_wrapper.call_trade("getActivatedEntities", ["location"])
        return obj

    def _goto_helper(self, x, y):
        logging.info(f"Setting goal: {x}, {y}")
        publisher = self.node.create_publisher(PoseListMessage, '/ihmc/continuous_walking/placed_goal_footsteps',
                                               self._qos_profile)
        poses = []
        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        poses.append(pose)

        pose = Pose()
        pose.position = Point(x=x, y=y + 0.3, z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        poses.append(pose)

        pose_message = PoseListMessage()
        pose_message.poses = poses
        publisher.publish(pose_message)

        command = ContinuousWalkingCommandMessage()
        command.publish_to_controller = True
        command.enable_continuous_walking = True
        self._walk_publisher.publish(command)

        # self.get_logger().info(f'Publishing {command}')

    @JOverride
    def getPoseGlobalQuat(self):
        pass

    @JOverride
    def setPoseGlobal(self, x, y, theta):
        pass

    @JOverride
    def goToLocation(self, location: Symbol, wait: bool) -> Justification:
        logging.info(f"[goToLocation] {location}, {wait}")
        return self._handle_return(True)

    @JOverride
    def goToLocation(self, desiredLocation: Symbol, initialLocation: Symbol) -> Justification:
        logging.info(f"[checkAt] {desiredLocation}, {initialLocation}")
        return self._handle_return(True)

    @JOverride
    def goToLocation(self, xdest, ydest, quat_x, quat_y, quat_z, quat_w, wait) -> Justification:
        logging.info(f"[checkAt] {xdest}")
        return self._handle_return(True)

    @JOverride
    def approachLocation(self, location) -> Justification:
        return self._handle_return(True)

    @JOverride
    def approachLocation(self, desiredLocation, initialLocation) -> Justification:
        return self._handle_return(True)

    @JOverride
    def stop(self):
        logging.info("[stop]")
        # command = ContinuousWalkingCommandMessage()
        # command.publish_to_controller = True
        # command.enable_continuous_walking = False
        # self._walk_publisher.publish(command)
        # return self._handle_return(True)

    @JOverride
    def isMoving(self):
        logging.info(f"[isMoving]")
        return self._handle_return(True)

    @JOverride
    def checkAt(self, locationTerm):
        logging.info(f"[checkAt] {locationTerm}")
        # Todo: Return something

    def _handle_return(self, value: bool, reason: Predicate = None):
        if reason is None:
            return ConditionJustification(value)
        return ConditionJustification(value, reason)

    @JOverride
    def goToLocation(self, location):
        logging.info(f"{location}")
        return self._handle_return(True)


def listen_for_input():
    """Function to listen for input commands."""
    logging.info("Listening for commands...")
    while True:
        command = input().strip()  # Read input from stdin
        if command.lower() == "shutdown":
            logging.info("Shutdown command received. Exiting...")
            break


if __name__ == '__main__':
    rclpy.init()

    # Set up TRADE
    trade_wrapper = TRADEWrapper()

    # # Set up Nadia
    try:
        nadia_wrapper = NadiaWrapper(trade_wrapper)
        TRADE.registerAllServices(nadia_wrapper, "")
        TRADE.registerAllServices(nadia_wrapper.consultant, nadia_wrapper.kb_name)
        nadia_wrapper.consultant.loadReferencesFromFile(
            "/config/edu/tufts/hrilab/map/refs/ihmcRefs.json") # relative path from resources classpath

        time.sleep(2)

        ros_thread = threading.Thread(target=rclpy.spin, args=(nadia_wrapper.node,))
        ros_thread.start()
        # input_thread = threading.Thread(target=listen_for_input, daemon=True)
        # input_thread.start()

        # todo: fail at push panel
        # todo: change to push failing

    except Exception as ex:
        logging.info(ex)
