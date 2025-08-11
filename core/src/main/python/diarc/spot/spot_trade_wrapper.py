import logging
import os
import time
import typing

from bosdyn.api.graph_nav import graph_nav_pb2, nav_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body

from bosdyn.api import manipulation_api_pb2, geometry_pb2
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client import frame_helpers
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.math_helpers import Vec3
from jpype import JImplements, JOverride, JClass
from spot_wrapper.wrapper import SpotWrapper
from diarc.pytrade.wrapper import TRADEWrapper
import yaml
from functools import wraps

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import DockingInterface, MoveBaseInterface, ArmInterface, SpotInterface
from edu.tufts.hrilab.spot.consultant import SpotNavGraphLocationConsultant
from javax.vecmath import Point3d, Matrix4d, Vector3d, Quat4d
from edu.tufts.hrilab.action.justification import ConditionJustification

logger = logging.getLogger(__name__)


def handle_return(func):
    """
    Convenience method for wrapping boolean returns in condition justifications.
    :param func: Function to wrap the return of
    :return:
    """
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        ret = func(self, *args, **kwargs)
        return ConditionJustification(ret[0])

    return wrapper


@JImplements([DockingInterface, MoveBaseInterface, ArmInterface, SpotInterface])
class SpotTRADEWrapper:
    def __init__(self, config, trade_wrapper: TRADEWrapper):
        self._reference_frame = "gpe"
        self.trade = trade_wrapper
        self._mapping = False
        self._recording_client = None
        self.spot = SpotWrapper(
            username=config['username'],
            password=config['password'],
            hostname=config['hostname'],
            port=config['port'],
            robot_name=config['name'],
            logger=logger,
            start_estop=config['start_estop'],
            estop_timeout=config['start_estop'],
            use_take_lease=config['use_take_lease'],
            get_lease_on_action=config['get_lease_on_action'],
            continually_try_stand=True,
            rgb_cameras=config['rgb_cameras'],
        )
        self.locationConsultant = SpotNavGraphLocationConsultant("location")
        # todo: Load existing refs
        loadRefs = False
        if loadRefs:
            refsConfigDir = ""
            refsConfigFile = ""
            self.locationConsultant.loadReferencesFromFile(refsConfigDir, refsConfigFile)

    @JOverride
    def moveTo(self, groupName, point, orientation):
        return self.moveTo(groupName.getName(), point.x, point.y, point.z, orientation.x, orientation.y, orientation.z,
                           orientation.w)

    @JOverride
    def moveTo(self, groupName, point_l, orientation_l, point_r, orientation_r):
        pass

    @JOverride
    def moveTo(self, groupName, refId):
        pass

    @JOverride
    def moveTo(self, groupName, refId, constraints):
        pass

    @JOverride
    def moveToRelative(self, groupName, point, orientation):
        pass

    @JOverride
    def graspObject(self, groupName, refId, position):
        pass

    @JOverride
    def releaseObject(self, groupName, refId, position):
        pass

    @JOverride
    def pointTo(self, groupName, objectRef):
        pass

    @JOverride
    def getEEPose(self, groupName):
        pass

    @JOverride
    def getPose(self, linkName):
        pass

    @JOverride
    def recordPose(self, poseName):
        pass

    @JOverride
    def saveEEPosesToFile(self, filename):
        pass

    @JOverride
    def loadEEPosesFromFile(self, filename):
        pass

    @JOverride
    def savePosesToFile(self, filename):
        pass

    @JOverride
    def loadPosesFromFile(self, filename):
        pass

    @JOverride
    def goToPose(self, groupName, poseName):
        pass

    @JOverride
    def goToPose(self, poseName):
        pass

    @JOverride
    def startRecordingTrajectory(self, trajectoryName):
        pass

    @JOverride
    def stopRecordingTrajectory(self):
        pass

    @JOverride
    def executeTrajectory(self, trajectoryName):
        pass

    @JOverride
    def saveTrajectoriesToFile(self, filename):
        pass

    @JOverride
    def loadTrajectoriesFromFile(self, filename):
        pass

    @handle_return
    @JOverride
    def closeGripper(self, groupName):
        return self.spot.spot_arm.gripper_close()

    @handle_return
    @JOverride
    def openGripper(self, groupName):
        return self.spot.spot_arm.gripper_open()

    @handle_return
    @JOverride
    def moveGripper(self, groupName, meters):
        # Todo: Ours is denoting distance, theirs is denoting angle. Do some conversion
        return self.spot.spot_arm.gripper_angle_open(meters)

    @JOverride
    def getGripperPosition(self, groupName):
        pass

    @JOverride
    def pressObject(self, group_name, object_location, object_orientation):
        pass

    @JOverride
    def pressObject(self, group_name, refID):
        pass

    ### Dock Interface
    @handle_return
    @JOverride
    def dock(self, dockId):
        if isinstance(dockId, int):
            return self.spot.spot_docking.dock(dockId)
        return self.spot.spot_docking.dock(dockId.getName())

    @handle_return
    @JOverride
    def undock(self):
        return self.spot.spot_docking.undock()

    ### Move Base Interface
    @JOverride
    def getPoseGlobalQuat(self):
        pass

    @JOverride
    def setPoseGlobal(self, x, y, theta):
        pass

    @JOverride
    def goToLocation(self, location, wait):
        pass

    @handle_return
    @JOverride
    def goToLocation(self, location):
        # todo: Handle consultant stuff
        waypoint_id = self.locationConsultant.getRefWithId(location.getName()).getGraphId()
        return self.spot.spot_graph_nav.navigate_to_existing_waypoint(waypoint_id)

    @JOverride
    def goToLocation(self, desiredLocation, initialLocation):
        # todo: get ininital loc
        return self.goToLocation(desiredLocation)

    @JOverride
    def goToLocation(self, xdest, ydest, quat_x, quat_y, quat_z, quat_w, wait):
        pass

    @JOverride
    def approachLocation(self, location):
        pass

    @JOverride
    def approachLocation(self, desiredLocation, initialLocation):
        pass

    @JOverride
    def goToStartPose(self, safe):
        pass

    @JOverride
    def getTransform(self, dstFrame):
        return self.getTransform(self._reference_frame, dstFrame)

    @JOverride
    def getTransform(self, srcFrame, dstFrame):
        tmp = Matrix4d()
        tmp.setIdentity()
        tf = get_a_tform_b(self.spot.robot_state.kinematic_state.frame_tree_snapshot, srcFrame, dstFrame)
        tmp.setRotation(Quat4d(tf.rot.x.tf.rot.y, tf.rot.z, tf.rot.w))
        tmp.setTranslation(Vector3d(tf.x, tf.y, tf.z))
        return tmp

    ### Spot Interface
    @JOverride
    def closeGripper(self):
        return self.closeGripper("")

    @JOverride
    def openGripper(self):
        return self.openGripper("")

    @handle_return
    @JOverride
    def stop(self):
        return self.spot.stop()

    @JOverride
    def isMoving(self):
        return self.spot.is_moving

    @JOverride
    def checkAt(self, locationTerm):
        pass

    @JOverride
    def command(self, input):
        pass

    @handle_return
    @JOverride
    def claim(self):
        return self.spot.claim()

    @handle_return
    @JOverride
    def release(self):
        return self.spot.release()

    @handle_return
    @JOverride
    def stand(self):
        return self.spot.stand()

    @handle_return
    @JOverride
    def sit(self):
        return self.spot.sit()

    @handle_return
    @JOverride
    def powerOn(self):
        return self.spot.power_on()

    @handle_return
    @JOverride
    def powerOff(self):
        return self.spot.safe_power_off()

    @handle_return
    @JOverride
    def stow(self):
        return self.spot.spot_arm.arm_stow()

    @handle_return
    @JOverride
    def unstow(self):
        return self.spot.spot_arm.arm_unstow()

    @handle_return
    @JOverride
    def carry(self):
        return self.spot.spot_arm.arm_carry()

    @handle_return
    @JOverride
    def pickUpObject(self, refId):
        source_frame = "hand_color_image_sensor"

        mo = self.trade.call_trade("getEntityForReference", refId,
                                   JClass("edu.tufts.hrilab.vision.stm.MemoryObject"))
        mo.transformToBase()
        start = Point3d(0, 0, 0)
        mo.getBaseTransform().transform(start)

        end = Point3d(start.x, start.y, start.z)
        direction = mo.getDirection()
        direction.scale(4.0)
        end.add(direction)
        grasp = manipulation_api_pb2.PickObjectRayInWorld(
            ray_start_rt_frame=Vec3(start.x, start.y, start.z).to_proto(),
            ray_end_rt_frame=Vec3(end.x, end.y, end.z).to_proto(),
            frame_name=self._reference_frame,
            walk_gaze_mode=2
        )

        # We can specify where in the gripper we want to grasp. About halfway is generally good for
        # small objects like this. For a bigger object like a shoe, 0 is better (use the entire
        # gripper)
        grasp.grasp_params.grasp_palm_to_fingertip = 0.2
        #
        # The axis on the gripper is the x-axis.
        axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

        # The axis in the vision frame is the negative z-axis
        axis_to_align_with_ewrt_vision = geometry_pb2.Vec3(x=0, y=0, z=-1)

        # Add the vector constraint to our proto.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
            axis_on_gripper_ewrt_gripper)
        constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
            axis_to_align_with_ewrt_vision)

        # We'll take anything within about 15 degrees for top-down or horizontal grasps.
        constraint.vector_alignment_with_tolerance.threshold_radians = 0.5

        # Specify the frame we're using.
        grasp.grasp_params.grasp_params_frame_name = frame_helpers.VISION_FRAME_NAME
        grasp.grasp_params.manipulation_camera_source = 2

        # Build the proto
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(
            pick_object_ray_in_world=grasp)

        return self.spot.manipulation_command(grasp_request)

    @handle_return
    @JOverride
    def setInitialLocation(self, refId):
        pass

    @handle_return
    @JOverride
    def setLocationWithFiducial(self, refId):
        pass

    @handle_return
    @JOverride
    def bindLocationToCurrentPose(self, refId):
        ref = self.locationConsultant.getRefWithId(refId)

    @JOverride
    @handle_return
    def goToLocationUnsafe(self, location):
        return self.goToLocation(location)

    @JOverride
    @handle_return
    def hasBeenLocalized(self):
        pass

    @JOverride
    def localize(self):
        # """Trigger localization when near a fiducial."""
        state = self.spot._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            state.kinematic_state.transforms_snapshot).to_proto()

        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self.spot._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                     ko_tform_body=current_odom_tform_body)

    @JOverride
    @handle_return
    def detectionPose(self):
        return self.spot.spot_arm.hand_pose(0.818, -0.003, 0.523, -0.000, 0.389, -0.004, 0.921)

    def start_mapping(self):
        """
        Puts the system in recording mode, which will allow the create of new waypoints with create_waypoint(...)
        :return:
        """
        if self._mapping:
            logging.info("Already mapping")
            return
        recording_client = self.spot._robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
        recording_client.start_recording()
        self._mapping = True

    def create_waypoint(self, name):
        """
        Creates a waypoint at the robot's current location. Requires start_mapping(...) to have been called.
        Note that this doesn't merge this waypoint until finish_mapping(...) is called.
        :param name: Human-readable name of the waypoint
        :return:
        """
        # todo: add to consultant here
        if not self._mapping:
            logging.info("Not mapping")
            return
        record_resp = self._recording_client.create_waypoint(waypoint_name=name)
        logging.info(f"New waypoint ID: {record_resp.created_waypoint}")

    def finish_mapping(self):
        """
        Turns off recording mode and merges any new waypoints into the current graph.
        TODO: There may be some anchor optimization that could happen here. If you see weird nav behaviors, I would check that out first.
        :return:
        """
        if not self._mapping:
            logging.info("Not mapping")
            return

        # Stop recording
        while True:
            try:
                self._recording_client.stop_recording()
                break
            except:
                time.sleep(1)

        self._mapping = False

        self.spot.spot_graph_nav.download_navigation_graph("/home/mfawn/new_map")
        self.spot.spot_graph_nav.upload_graph("/home/mfawn/new_map")
        spot_wrapper.spot.spot_graph_nav.list_graph()  # Have to call this to update map


# This main is largely used for testing behaviors
if __name__ == '__main__':
    with open('/home/mfawn/code/diarc/diarc/core/src/main/python/spot/spot_phi.yaml', 'r') as file:
        config = yaml.safe_load(file)
        config = config['/**']['ros__parameters']

    trade_wrapper = TRADEWrapper()

    spot_wrapper = SpotTRADEWrapper(config, trade_wrapper)
    # spot_wrapper = MockSpotTRADEWrapper(config, trade_wrapper)
    # TRADE.registerAllServices(spot_wrapper, "")
    # TRADE.registerAllServices(spot_wrapper.locationConsultant, "location")
    # time.sleep(1)

    spot_wrapper.claim()
    spot_wrapper.spot.spot_graph_nav.upload_graph("/home/mfawn/new_map")
    spot_wrapper.spot.spot_graph_nav.set_initial_localization_fiducial()

    # spot_wrapper.undock()
    pts = spot_wrapper.spot.spot_graph_nav.list_graph()  # Have to call this to update map
    logging.info(spot_wrapper.spot.spot_graph_nav._current_annotation_name_to_wp_id["test_two"])

    # spot_wrapper.dock(520)
