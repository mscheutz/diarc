import threading
import time

import rclpy
from pyTRADE.core.wrapper import TRADEWrapper

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import IHMCInterface
from jpype import JImplements, JOverride
from geometry_msgs.msg import Point, Quaternion, Pose
from ihmc_common_msgs.msg import PoseListMessage
from behavior_msgs.msg import ContinuousWalkingCommandMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

locations = {"home": (0.0, 0.0),
             "pose1": (3.0, 0.0),
             "pose2": (0.0, 3.0)}


@JImplements(IHMCInterface)
class NadiaWrapper:
    def __init__(self):
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
    def stop(self):
        command = ContinuousWalkingCommandMessage()
        command.publish_to_controller = True
        command.enable_continuous_walking = False
        self._walk_publisher.publish(command)

    @JOverride
    def go_to(self, x, y):
        self._goto_helper(x, y)

    def _goto_helper(self, x, y):
        print(f"Setting goal: {x}, {y}")
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
    def go_to(self, location):
        print(f"Setting goal: {location}")
        loc = locations.get(location)
        x = loc[0]
        y = loc[1]
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
        print(f"command {command}")

        self._walk_publisher.publish(command)
        print(f"published walk")

        # self.get_logger().info(f'Publishing {command}')


if __name__ == '__main__':
    rclpy.init()

    # Set up TRADE
    trade_wrapper = TRADEWrapper()

    # # Set up Nadia
    nadia_wrapper = NadiaWrapper()
    TRADE.registerAllServices(nadia_wrapper, "")
    ros_thread = threading.Thread(target=rclpy.spin, args=(nadia_wrapper.node,))
    ros_thread.start()

    time.sleep(2)
    print(TRADE.getAvailableServices())
    # nadia_wrapper.set_goal()
    # nadia_wrapper.walk()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        nadia_wrapper.node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()
