import logging
import time
import argparse
import traceback

from bosdyn.api.geometry_pb2 import Vec2
from bosdyn.client.robot_command import RobotCommandBuilder

from bosdyn.api.graph_nav import graph_nav_pb2, nav_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body, GROUND_PLANE_FRAME_NAME, BODY_FRAME_NAME, HAND_FRAME_NAME, \
    ODOM_FRAME_NAME
from bosdyn.api import manipulation_api_pb2, geometry_pb2
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client import frame_helpers, math_helpers
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.math_helpers import Vec3
from jpype import JImplements, JOverride, JClass, JString, JObject, JBoolean, JArray

from spot_wrapper.wrapper import SpotWrapper
from diarc.pytrade.wrapper import TRADEWrapper
import yaml
from functools import wraps

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.interfaces import DockingInterface, MoveBaseInterface, ArmInterface, SpotInterface, CoordinateFramesInterface
from edu.tufts.hrilab.spot.consultant import SpotNavGraphLocationConsultant
from edu.tufts.hrilab.fol import Factory, Symbol, Predicate
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


@JImplements([DockingInterface, MoveBaseInterface, ArmInterface, SpotInterface, CoordinateFramesInterface])
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
            port=config.get('port', ''),
            robot_name=config.get('name', ''),
            logger=logger,
            start_estop=config['start_estop'],
            estop_timeout=config['estop_timeout'],
            use_take_lease=config['use_take_lease'],
            get_lease_on_action=config.get('get_lease_on_action',True),
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

    def _get_center_pixel(self, refId: Symbol) -> (int, int):
        # get pixel coordinates from refId
        mo = self.trade.call_trade("getEntityForReference", refId,
                                   JClass("edu.tufts.hrilab.vision.stm.MemoryObject"))
        if mo is None:
            return False

        x_pix = mo.getBoundingBox().x
        y_pix = mo.getBoundingBox().y
        return (x_pix, y_pix)

    def _get_transforms_snapshot(self):
        return self.spot._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    ###################### moveTo overloaded method handling

    @handle_return
    @JOverride
    def moveTo(self, *args):
        logging.info(f"moveTo dispatch method called")
        # always use the wild card args when implementing a dispatch

        if len(args)==2 and isinstance(args[0], JString) and isinstance(args[1], Symbol):
            return self.moveToRef(*args)
        if len(args)==3 and isinstance(args[0], JString) and isinstance(args[1], Point3d) and isinstance(args[2], Quat4d):
            return self.moveToPose(*args)

        logging.info(f"No valid argument match found.")
        raise RuntimeError("Incorrect arguments")

    def moveToPose(self, groupName, point, orientation):
        logging.info(f"moveToPose method called. point: {point} orientation: {orientation}")

        try:
            # Build the SE(3) pose of the desired hand position in the moving body frame.
            body_T_hand = geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=point.x, y=point.y, z=point.z),
                rotation=geometry_pb2.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
            )

            # Transform the desired from the moving body frame to the odom frame.
            odom_T_body = get_a_tform_b(self._get_transforms_snapshot(),
                                        ODOM_FRAME_NAME, BODY_FRAME_NAME)
            odom_T_hand = odom_T_body * math_helpers.SE3Pose.from_proto(body_T_hand)

            command = RobotCommandBuilder.arm_pose_command_from_pose(odom_T_hand.to_proto(), ODOM_FRAME_NAME, build_on_command=RobotCommandBuilder.follow_arm_command())
            (success, msg, cmd_id) = self.spot.robot_command(command)
            # self.spot.spot_arm.wait_for_arm_command_to_complete(cmd_id, timeout_sec=3.0)
            return (success, msg)

        except Exception as e:
            logging.error(f"Error calling moveToPose: {traceback.format_exc()}")
            return (False, "error msg")

    def moveToRef(self, groupName, refId: JObject):
        logging.info(f"moveToRef method called. refId: {refId}")
        return (True, "")

    ####################################################

    @JOverride
    def moveToRelative(self, groupName, point, orientation):
        try:
            target_pose = geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=point.x, y=point.y, z=point.z),
                rotation=geometry_pb2.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
            )

            command = RobotCommandBuilder.arm_pose_command_from_pose(target_pose, HAND_FRAME_NAME, build_on_command=RobotCommandBuilder.follow_arm_command())
            (success, msg, cmd_id) = self.spot.robot_command(command)
            # self.spot.spot_arm.wait_for_arm_command_to_complete(cmd_id, timeout_sec=3.0)
            return (success, msg)

        except Exception as e:
            logging.error(f"Error calling moveToRelative: {traceback.format_exc()}")
            return (False, "error msg")

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

    @handle_return
    @JOverride
    def goToPose(self, *args):
        logging.info(f"goToPose dispatch method called")
        # always use the wild card args when implementing a dispatch

        if len(args)==1:
            return self.goToPose1(*args)
        if len(args)==2:
            return self.goToPose2(*args)

        logging.info(f"No valid argument match found.")
        raise RuntimeError("Incorrect arguments")

    def goToPose1(self, poseName):
        logging.info(f"goToPose1 called: {poseName}")
        pose = str(poseName.getName())
        if pose == 'carry':
            return self.spot.spot_arm.arm_carry()
        if pose == 'detection':
            return self.spot.spot_arm.hand_pose(0.818, -0.003, 0.523, -0.000, 0.389, -0.004, 0.921)
        if pose == 'stow':
            return self.spot.spot_arm.arm_stow()
        if pose == 'unstow':
            return self.spot.spot_arm.arm_unstow()

        return False, ""

    def goToPose2(self, groupName, poseName):
        logging.info(f"goToPose2 called: {groupName} {poseName}")
        return self.goToPose1(poseName)

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
        try:
            return self.spot.spot_arm.gripper_close()
        except Exception as e:
            logging.error(f"Error calling closeGripper: {traceback.format_exc()}")
            return (False, "error msg")

    @handle_return
    @JOverride
    def openGripper(self, groupName):
        try:
            return self.spot.spot_arm.gripper_open()
        except Exception as e:
            logging.error(f"Error calling openGripper: {traceback.format_exc()}")
            return (False, "error msg")

    @handle_return
    @JOverride
    def moveGripper(self, groupName, meters):
        # Todo: Ours is denoting distance, theirs is denoting angle. Do some conversion
        return self.spot.spot_arm.gripper_angle_open(meters)

    @JOverride
    def getGripperPosition(self, groupName):
        pass

    ###################### pressObject overloaded method handling
    @handle_return
    @JOverride
    def pressObject(self, *args):
        logging.info(f"pressObject dispatch method called")
        # always use the wild card args when implementing a dispatch

        if len(args)==2:
            return self.pressObjectRef(*args)
        if len(args)==3:
            return self.pressObjectPose(*args)

        logging.info(f"No valid argument match found.")
        raise RuntimeError("Incorrect arguments")

    def pressObjectRef(self, group_name, refID):
        pass

    def pressObjectPose(self, group_name, object_location, object_orientation):
        # construct offset pose (set distance away from object_location along object_orientation vector)

        # move gripper to offset pose
        # moveToPose()

        # move gripper into pressing position

        # move gripper back to offset pose
        pass

    ###########################################################

    ### Dock Interface
    @handle_return
    @JOverride
    def dock(self, dockId):
        logging.info(f"dock called with id: {dockId}")
        try:
            if isinstance(dockId, Symbol):
                return self.spot.spot_docking.dock(int(str(dockId.getName())))
            if isinstance(dockId, int):
                return self.spot.spot_docking.dock(dockId)
        except Exception as e:
            logging.error(f"Error calling dock: {traceback.format_exc()}")

        return (False, "")

    @handle_return
    @JOverride
    def undock(self):
        try:
            return self.spot.spot_docking.undock()
        except Exception as e:
            logging.error(f"Error calling undock: {traceback.format_exc()}")
            return (False, "error msg")

    ### Move Base Interface
    @JOverride
    def getPoseGlobalQuat(self):
        pass

    @JOverride
    def setPoseGlobal(self, x, y, theta):
        pass

    ###################### goToLocation overloaded method handling
    @handle_return
    @JOverride
    def goToLocation(self, *args):
        # dispatch method for overloaded goToLocation

        if len(args)==1:
            return self.goToLocationRef(args[0])
        if len(args)==2 and isinstance(args[0], Symbol) and isinstance(args[1], JBoolean):
            return self.goToLocationRefWait(*args)
        if len(args)==2 and isinstance(args[0], Symbol) and isinstance(args[1], Symbol):
            return self.goToLocationFrom(*args)
        if len(args)==7:
            return self.goToLocationPose(*args)

        logging.info(f"No valid argument match found.")
        raise RuntimeError("Incorrect arguments")

    def goToLocationRef(self, location):
        # todo: Handle consultant stuff
        waypoint_id = self.locationConsultant.getRefWithId(location.getName()).getGraphId()
        return self.spot.spot_graph_nav.navigate_to_existing_waypoint(waypoint_id)

    def goToLocationRefWait(self, location, wait):
        pass

    def goToLocationFrom(self, desiredLocation, initialLocation):
        # todo: get ininital loc
        return self.goToLocation(desiredLocation)

    def goToLocationPose(self, xdest, ydest, quat_x, quat_y, quat_z, quat_w, wait):
        self.spot.spot_images.get_rgb_image()
        pass

    #####################################################################

    @handle_return
    @JOverride
    def approachLocation(self, refId):

        # TODO: choose ray vs pixel based on contents in MO (i.e., if mo.getLocation() has nan values)
        walk_to_object_in_image = False
        if walk_to_object_in_image:
            # use 2D pixel based bounding box to specify object to grasp -- assumes spot hand camera
            (x_pix, y_pix) = self._get_center_pixel(refId)
            image_response = self.spot.spot_images.get_hand_rgb_image()
            walk = manipulation_api_pb2.WalkToObjectInImage(
                pixel_xy=Vec2(x_pix, y_pix),
                transforms_snapshot_for_camera=self._get_transforms_snapshot(),
                frame_name_image_sensor=image_response.shot.frame_name_image_sensor,
                camera_model=image_response.source.pinhole,
            )
        else:
            # use ray to specify object to grasp -- requires 3D camera data
            mo = self.trade.call_trade("getEntityForReference", refId,
                                       JClass("edu.tufts.hrilab.vision.stm.MemoryObject"))
            mo.transformToBase()
            start = Point3d(0, 0, 0)
            mo.getBaseTransform().transform(start)

            end = Point3d(start.x, start.y, start.z)
            direction = mo.getDirection()
            direction.scale(4.0)
            end.add(direction)
            walk = manipulation_api_pb2.WalkToObjectRayInWorld(
                ray_start_rt_frame=Vec3(start.x, start.y, start.z),
                ray_end_rt_frame=Vec3(end.x, end.y, end.z),
                frame_name=self._reference_frame,
            )

        # Build the proto
        if walk_to_object_in_image:
            walk_request = manipulation_api_pb2.ManipulationApiRequest(
                walk_to_object_in_image=walk)
        else:
            walk_request = manipulation_api_pb2.ManipulationApiRequest(
                walk_to_object_ray_in_world=walk)

        return self.spot.manipulation_command(walk_request)[:1]

    @JOverride
    def goToStartPose(self, safe):
        pass

    @JOverride
    def getTransform(self, srcFrame, dstFrame):
        try:
            src_frame_name = str(srcFrame)
            dst_frame_name = str(dstFrame)
            transforms_snapshot = self._get_transforms_snapshot()

            if dst_frame_name == "hand_color_image_sensor":
                # get hand_color_image_sensor to body
                transforms_snapshot_with_sensor = self.spot.spot_images.get_hand_rgb_image().shot.transforms_snapshot
                src_tf_body = get_a_tform_b(transforms_snapshot, src_frame_name, BODY_FRAME_NAME)
                body_tf_dst = get_a_tform_b(transforms_snapshot_with_sensor, BODY_FRAME_NAME, dst_frame_name)
                src_tf_dst = src_tf_body * body_tf_dst
            elif src_frame_name == "hand_color_image_sensor":
                # get body to hand_color_image_sensor
                transforms_snapshot_with_sensor = self.spot.spot_images.get_hand_rgb_image().shot.transforms_snapshot
                src_tf_body = get_a_tform_b(transforms_snapshot_with_sensor, src_frame_name, BODY_FRAME_NAME)
                body_tf_dst = get_a_tform_b(transforms_snapshot, BODY_FRAME_NAME, dst_frame_name)
                src_tf_dst = src_tf_body * body_tf_dst
            else:
                src_tf_dst = get_a_tform_b(transforms_snapshot, src_frame_name, dst_frame_name)

            # finally, get transform
            tmp = Matrix4d()
            tmp.setIdentity()
            tmp.setRotation(Quat4d(src_tf_dst.rotation.x, src_tf_dst.rotation.y, src_tf_dst.rotation.z, src_tf_dst.rotation.w))
            tmp.setTranslation(Vector3d(src_tf_dst.position.x, src_tf_dst.position.y, src_tf_dst.position.z))
            return tmp

        except Exception as e:
            logging.error(f"Error calling getTransform: {traceback.format_exc()}")
            return Matrix4d()

    @JOverride
    def getCoordinateFrames(self):
        try:
            # TODO: combine these lists
            # frame_names = frame_helpers.get_frame_names( self.spot.spot_images.get_hand_rgb_image().shot.transforms_snapshot)
            frame_names = frame_helpers.get_frame_names(self._get_transforms_snapshot())
            logging.info(f"Found frame names: {frame_names}")
            java_list = JArray(JString)(frame_names)

        except Exception as e:
            logging.error(f"Error calling getCoordinateFrames: {traceback.format_exc()}")
            return ()

        return java_list

    @JOverride
    def addLocalStaticTransform(self):
        pass

    ### Spot Interface

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
        try:
            return self.spot.stand()
        except Exception as e:
            logging.error(f"Error calling stand: {e}")
            return (False, "error msg")

    @handle_return
    @JOverride
    def sit(self):
        logging.info("python sit called")
        return self.spot.sit()

    @handle_return
    @JOverride
    def powerOn(self):
        logging.info("python powerOn called")
        return self.spot.power_on()

    @handle_return
    @JOverride
    def powerOff(self):
        return self.spot.safe_power_off()

    @handle_return
    @JOverride
    def pickUpObject(self, refId):

        # TODO: choose ray vs pixel based on contents in MO (i.e., if mo.getLocation() has nan values)
        pick_object_in_image = False
        if pick_object_in_image:
            # use 2D pixel based bounding box to specify object to grasp -- assumes spot hand camera
            (x_pix, y_pix) = self._get_center_pixel(refId)
            image_response = self.spot.spot_images.get_hand_rgb_image()
            grasp = manipulation_api_pb2.PickObjectInImage(
                pixel_xy=Vec2(x_pix, y_pix),
                transforms_snapshot_for_camera=self._get_transforms_snapshot(),
                frame_name_image_sensor=image_response.shot.frame_name_image_sensor,
                camera_model=image_response.source.pinhole,
                walk_gaze_mode=2
            )
        else:
            # use ray to specify object to grasp -- requires 3D camera data
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
                ray_start_rt_frame=Vec3(start.x, start.y, start.z),
                ray_end_rt_frame=Vec3(end.x, end.y, end.z),
                frame_name=self._reference_frame,
                walk_gaze_mode=2
            )

        # setting grasp params restricts the types of objects that can be picked up (top-down vs side, etc),
        # so it doesn't make for a very general pickUpObject method
        set_grasp_params = False
        if set_grasp_params:
            # We can specify where in the gripper we want to grasp. About halfway is generally good for
            # small objects like this. For a bigger object like a shoe, 0 is better (use the entire
            # gripper)
            grasp.grasp_params.grasp_palm_to_fingertip = 0.2

            # align user defined gripper axis to user defined alignment axis
            constraint = grasp.grasp_params.allowable_orientation.add()
            # The axis on the gripper
            constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)
            # The axis to align the gripper axis to
            constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame = geometry_pb2.Vec3(x=0, y=0, z=-1)

            # We'll take anything within about 15 degrees for top-down or horizontal grasps.
            constraint.vector_alignment_with_tolerance.threshold_radians = 0.5


        # Specify the frame we're using.
        grasp.grasp_params.grasp_params_frame_name = GROUND_PLANE_FRAME_NAME
        grasp.grasp_params.manipulation_camera_source = 2

        # Build the proto
        if pick_object_in_image:
            grasp_request = manipulation_api_pb2.ManipulationApiRequest(
                pick_object_in_image=grasp)
        else:
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
    def hasBeenLocalized(self):
        pass

    @JOverride
    def localize(self):
        """Trigger localization when near a fiducial."""
        current_odom_tform_body = get_odom_tform_body(self._get_transforms_snapshot()).to_proto()

        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self.spot.spot_graph_nav.set_localization(initial_guess_localization=localization,
                                                     ko_tform_body=current_odom_tform_body)

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

        self.spot.spot_graph_nav.download_navigation_graph("/tmp/spot_map")
        self.spot.spot_graph_nav.upload_graph("/tmp/spot_map")
        spot_wrapper.spot.spot_graph_nav.list_graph()  # Have to call this to update map


# This main is largely used for testing behaviors
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Spot TRADE Wrapper')
    parser.add_argument('--spot-config', required=True, help='Path to Spot yaml config file')
    parser.add_argument('--spot-map', help='Path to Spot graph_nav file')
    parser.add_argument('--groups', nargs='*', help='TRADE group(s)')
    args = parser.parse_args()

    with open(args.spot_config, 'r') as file:
        config = yaml.safe_load(file)
        config = config['/**']['ros__parameters']

    trade_wrapper = TRADEWrapper()

    spot_wrapper = SpotTRADEWrapper(config, trade_wrapper)
    logging.info(f"TRADE groups: {args.groups}")
    TRADE.registerAllServices(spot_wrapper, args.groups if args.groups else "") # TODO: fix the args.groups when more than one group is specified
    TRADE.registerAllServices(spot_wrapper.locationConsultant, "location")

    # claim robot
#     spot_wrapper.claim()
#     spot_wrapper.spot.power_on()
#     spot_wrapper.spot.spot_arm.gripper_open()

    # load pre-built spot map
    if args.spot_map:
        logging.info(f"Loading spot map: {args.spot_map}")
        spot_wrapper.spot.spot_graph_nav.upload_graph(args.spot_map)
        spot_wrapper.spot.spot_graph_nav.set_initial_localization_fiducial()

        # spot_wrapper.undock() # TODO: is undock-ing needed here?
        pts = spot_wrapper.spot.spot_graph_nav.list_graph()  # Have to call this to update map
        logging.info(spot_wrapper.spot.spot_graph_nav._current_annotation_name_to_wp_id["test_two"])
