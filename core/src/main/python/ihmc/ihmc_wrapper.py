import logging
import time
import sys
import threading

import rclpy
from pytrade.wrapper import TRADEWrapper

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import IHMCInterface
from edu.tufts.hrilab.fol import Factory, Symbol
from edu.tufts.hrilab.action.justification import ConditionJustification, Justification

from jpype import JImplements, JOverride
from geometry_msgs.msg import Point, Quaternion, Pose
from ihmc_common_msgs.msg import PoseListMessage
from behavior_msgs.msg import ContinuousWalkingCommandMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

locations = {"home": (0.0, 0.0),
             "pose1": (3.0, 0.0),
             "pose2": (0.0, 3.0)}

logging.basicConfig(stream=sys.stdout, level=logging.INFO)
logging.info("This will show up in Java output")


@JImplements(IHMCInterface)
class NadiaWrapper:
    def __init__(self, trade_wrapper: TRADEWrapper):
        self.trade_wrapper = trade_wrapper
        self.node = rclpy.create_node('diarc_node')
        self._qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._walk_publisher = self.node.create_publisher(ContinuousWalkingCommandMessage,
                                                          '/ihmc/continuous_walking/command',
                                                          qos_profile=self._qos_profile)
        self._walk_timer = None

    @JOverride
    def walk(self):
        command = ContinuousWalkingCommandMessage()
        command.publish_to_controller = True
        command.enable_continuous_walking = True
        self._walk_publisher.publish(command)

    @JOverride
    def open_door(self):
        logging.info("Opening door")
        # Get door handle coords
        # Insert handle to scene graph
        # Call behavior tree
        pass

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
    def goToLocation(self, location, wait):
        try:
            logging.info(f"[goToLocation] {location}, {wait}")
        except Exception as ex:
            logging.info(f"Caught exception: {str(ex)}")
            raise(ex)
        return self._handle_return(True)

    @JOverride
    def goToLocation(self, desiredLocation, initialLocation):
        return self._handle_return(True)

    @JOverride
    def goToLocation(self, xdest, ydest, quat_x, quat_y, quat_z, quat_w, wait):
        return self._handle_return(True)

    @JOverride
    def approachLocation(self, location):
        return self._handle_return(True)

    @JOverride
    def approachLocation(self, desiredLocation, initialLocation):
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
        return self._handle_return(True)

    @JOverride
    def checkAt(self, locationTerm):
        print(f"[checkAt] {locationTerm}")
        return [{}]

    def _handle_return(self, value: bool):
        return ConditionJustification(value)

    @JOverride
    def goToLocation(self, location):
        logging.info(f"{location}")
        return self._handle_return(True)

if __name__ == '__main__':
    rclpy.init()

    # Set up TRADE
    trade_wrapper = TRADEWrapper()

    # # Set up Nadia
    try:
        nadia_wrapper = NadiaWrapper(trade_wrapper)
        TRADE.registerAllServices(nadia_wrapper, "")
        time.sleep(2)
        print(TRADE.getAvailableServices())
        nadia_wrapper.open_door()
        trade_wrapper.call_trade("open_door", "")
    except Exception as ex:
        logging.info(ex)
    # refId = Factory.createSymbol("location_1", "location")
    #
    # ros_thread = threading.Thread(target=rclpy.spin, args=(nadia_wrapper.node,))
    # ros_thread.start()

    # # nadia_wrapper.set_goal()
    # # nadia_wrapper.walk()
    # try:
    #     while True:
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     nadia_wrapper.node.destroy_node()
    #     rclpy.shutdown()
    #     ros_thread.join()
