import logging
import time
import sys
import threading

from pytrade.java_util import to_java_object, to_java_object_from_class
from pytrade.wrapper import TRADEWrapper

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import CoordinateFramesInterface
from edu.tufts.hrilab.action.justification import ConditionJustification, Justification
from jpype import JImplements, JOverride, JClass, JString
from javax.vecmath import Point3d, Matrix4d, Vector3d, Quat4d

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


@JImplements(CoordinateFramesInterface)
class TF2Wrapper(Node):
    def __init__(self, trade_wrapper: TRADEWrapper):
        super().__init__('diarc_tf2_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.trade_wrapper = trade_wrapper

    def _get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time()
            )
            return transform
        except tf2_ros.LookupException as e:
            logging.error(f"Lookup Exception: {e}")
            return None
        except tf2_ros.ConnectivityException as e:
            logging.error(f"Connectivity Exception: {e}")
            return None
        except tf2_ros.ExtrapolationException as e:
            logging.error.error(f"Extrapolation Exception: {e}")
            return None

    @JOverride
    def getTransform(self, source_frame, target_frame):
        tf = self._get_transform(str(target_frame), str(source_frame))

        if tf is None:
            return None

        translation = tf.transform.translation
        rotation = tf.transform.rotation

        tmp = Matrix4d()
        tmp.setIdentity()
        tmp.setRotation(Quat4d(rotation.x, rotation.y, rotation.z, rotation.w))
        tmp.setTranslation(Vector3d(translation.x, translation.y, translation.z))
        return tmp

    @JOverride
    def getCoordinateFrames(self):
        pass

    @JOverride
    def addLocalStaticTransform(self, parent, child, transform):
        pass

if __name__ == '__main__':
    rclpy.init()

    # Set up TRADE
    trade_wrapper = TRADEWrapper()

    # # Set up TF2
    try:
        tf2_wrapper = TF2Wrapper(trade_wrapper)
        TRADE.registerAllServices(tf2_wrapper, "")

        time.sleep(2)

        ros_thread = threading.Thread(target=rclpy.spin, args=(tf2_wrapper,))
        ros_thread.start()

        # todo: fail at push panel
        # todo: change to push failing

    except Exception as ex:
        logging.info(ex)
