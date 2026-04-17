import logging
import time
import argparse
import traceback

from bosdyn.api.geometry_pb2 import Vec2
from bosdyn.api.robot_state_pb2 import ManipulatorState
from bosdyn.client.robot_command import RobotCommandBuilder

from google.protobuf.wrappers_pb2 import DoubleValue

from bosdyn.api.graph_nav import graph_nav_pb2, nav_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body, GROUND_PLANE_FRAME_NAME, BODY_FRAME_NAME, HAND_FRAME_NAME, \
    ODOM_FRAME_NAME
from bosdyn.api import manipulation_api_pb2, geometry_pb2, basic_command_pb2, arm_command_pb2, robot_command_pb2, \
    trajectory_pb2, synchronized_command_pb2
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client import frame_helpers, math_helpers
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.util import seconds_to_duration
from jpype import JImplements, JOverride, JClass, JString, JObject, JBoolean, JArray

from spot_wrapper.wrapper import SpotWrapper
from diarc.pytrade.wrapper import TRADEWrapper
import yaml
from functools import wraps

from ai.thinkingrobots.trade import TRADE
from edu.tufts.hrilab.spot import SpotInterface
from edu.tufts.hrilab.spot.consultant import SpotLocationConsultant
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
        if ret[0] is False:
            logging.info(f"Failure msg: {ret[1]}")
        return ConditionJustification(ret[0])

    return wrapper


@JImplements([SpotInterface])
class SpotTRADEWrapper:
    def __init__(self, trade_wrapper: TRADEWrapper, config, refs_file: str = None):
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

        # Load existing refs
        self.locationConsultant = SpotLocationConsultant("location")
        logging.info(f"Loading location references from file: {refs_file}")
        if refs_file is not None:
            self.locationConsultant.loadReferencesFromFile(refs_file)

    def _get_center_pixel(self, refId: Symbol, img_src_width: int, img_src_height: int) -> (int, int):
        # get pixel coordinates from refId
        ret = self.trade.call_trade("getEntityForReference", [], refId, JClass("edu.tufts.hrilab.vision.stm.MemoryObject"))
        mo = JClass("edu.tufts.hrilab.vision.stm.MemoryObject")(ret)
        if mo is None:
            return None

        bbox = mo.getBoundingBox()
        x_pix = bbox.x + bbox.width / 2
        y_pix = bbox.y + bbox.height / 2

        logging.debug(f"BBox (x,y,width,height): {bbox.x}, {bbox.y}, {bbox.width}, {bbox.height}")
        logging.debug(f"Center pixel: {x_pix}, {y_pix}. Img src size: {img_src_width}, {img_src_height}")
        mo_img_width = int(mo.getImageWidth())
        mo_img_height = int(mo.getImageHeight())
        logging.info(f"mo img size: {mo_img_width}, {mo_img_height}")

        # scale pixel location to match image_response size
        #x_pix = x_pix * (img_src_width / float(mo.getImageWidth()))
        #y_pix = y_pix * (img_src_height / float(mo.getImageHeight()))
        x_pix = x_pix * (img_src_width / 320)
        y_pix = y_pix * (img_src_height / 240)

        logging.debug(f"Scaled center pixel: {x_pix}, {y_pix}.")

        return x_pix, y_pix

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
            return self.moveToPoseImpedanceControl(*args)
            # return self.moveToPose(*args)

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
            odom_T_body = get_a_tform_b(self._get_transforms_snapshot(), ODOM_FRAME_NAME, BODY_FRAME_NAME)
            odom_T_hand = odom_T_body * math_helpers.SE3Pose.from_proto(body_T_hand)

            command = RobotCommandBuilder.arm_pose_command_from_pose(odom_T_hand.to_proto(), ODOM_FRAME_NAME)
            (success, msg, cmd_id) = self.spot.robot_command(command)

            self.spot.spot_arm.wait_for_arm_command_to_complete(cmd_id, timeout_sec=3.0)
            return (success, msg)

        except Exception as e:
            logging.error(f"Error calling moveToPose: {traceback.format_exc()}")
            return (False, "error msg")

    def moveToPoseImpedanceControl(self, groupName, point, orientation):
        logging.debug(f"moveToPoseImpedanceControl method called. point: {point} orientation: {orientation}")

        try:
            # Set up stiffness and damping matrices. Note: if these values are set too high,
            # the arm can become unstable. Currently, these are the max stiffness and
            # damping values that can be set.
            stiffness = geometry_pb2.Vector(values=[500, 500, 500, 60, 60, 60])
            damping = geometry_pb2.Vector(values=[2.5, 2.5, 2.5, 1.0, 1.0, 1.0])

            # set up hand trajectory
            duration = seconds_to_duration(3.0)
            hand_pose = geometry_pb2.SE3Pose(
                    position=geometry_pb2.Vec3(x=point.x, y=point.y, z=point.z),
                    rotation=geometry_pb2.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
                )
            hand_pose_traj_point = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose,
                                                                     time_since_reference=duration)
            hand_trajectory = trajectory_pb2.SE3Trajectory(points=[hand_pose_traj_point])

            # built arm impedance command
            impedance_cmd = arm_command_pb2.ArmImpedanceCommand.Request(
                root_frame_name=BODY_FRAME_NAME, task_tform_desired_tool=hand_trajectory,
                diagonal_stiffness_matrix=stiffness, diagonal_damping_matrix=damping,
                max_force_mag=DoubleValue(value=20.0), max_torque_mag=DoubleValue(value=20.0))
            arm_command = arm_command_pb2.ArmCommand.Request(
                arm_impedance_command=impedance_cmd)
            synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
                arm_command=arm_command)
            robot_command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

            (success, msg, cmd_id) = self.spot.robot_command(robot_command)
            self.spot.spot_arm.wait_for_arm_command_to_complete(cmd_id, timeout_sec=5.0)

            return (success, msg)

        except Exception as e:
            logging.error(f"Error calling moveToPoseImpedanceControl: {traceback.format_exc()}")
            return (False, "error msg")

    # def moveToPoseWithForceLimits(self, groupName, point, orientation):
    #     logging.info(f"moveToPoseWithForceLimits method called. point: {point} orientation: {orientation}")
    #
    #     try:
    #         frame_name = BODY_FRAME_NAME
    #         control_mode = basic_command_pb2.ConstrainedManipulationCommand.Request.CONTROL_MODE_POSITION
    #         force_lim = 5.0
    #         torque_lim = 5.0
    #
    #         curr_body_T_hand = get_a_tform_b(self._get_transforms_snapshot(),  BODY_FRAME_NAME, HAND_FRAME_NAME)
    #         curr_hand_pos = curr_body_T_hand.position
    #         target_hand_pos = geometry_pb2.Vec3(x=point.x, y=point.y, z=point.z)
    #         target_dir = target_hand_pos - curr_hand_pos
    #
    #         # first, perform translation
    #         position_value = target_dir.length()
    #         force_direction = target_dir
    #         torque_direction = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    #         init_wrench_dir = geometry_pb2.Wrench(force=force_direction, torque=torque_direction)
    #         task_type = basic_command_pb2.ConstrainedManipulationCommand.Request.TASK_TYPE_R3_LINEAR_FORCE
    #
    #         command = RobotCommandBuilder.constrained_manipulation_command(
    #             task_type=task_type, init_wrench_direction_in_frame_name=init_wrench_dir,
    #             force_limit=force_lim, torque_limit=torque_lim,
    #             frame_name=frame_name, control_mode=control_mode, target_linear_position=position_value,
    #             reset_estimator=True)
    #
    #         (success, msg, cmd_id) = self.spot.robot_command(command)
    #         # self.spot.spot_arm.wait_for_arm_command_to_complete(cmd_id, timeout_sec=3.0)
    #
    #
    #         # second, perform rotation
    #         force_direction = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    #         torque_direction = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0) # TODO: calculate this
    #         init_wrench_dir = geometry_pb2.Wrench(force=force_direction, torque=torque_direction)
    #         task_type = basic_command_pb2.ConstrainedManipulationCommand.Request.TASK_TYPE_SE3_ROTATIONAL_TORQUE
    #         angle_value = # TODO: calculate this
    #
    #         command = RobotCommandBuilder.constrained_manipulation_command(
    #             task_type=task_type, init_wrench_direction_in_frame_name=init_wrench_dir,
    #             force_limit=force_lim, torque_limit=torque_lim,
    #             frame_name=frame_name, control_mode=control_mode, target_angle=angle_value,
    #             reset_estimator=True)
    #
    #         (success, msg, cmd_id) = self.spot.robot_command(command)
    #         # self.spot.spot_arm.wait_for_arm_command_to_complete(cmd_id, timeout_sec=3.0)
    #
    #         return (success, msg)
    #
    #     except Exception as e:
    #         logging.error(f"Error calling moveToPose: {traceback.format_exc()}")
    #         return (False, "error msg")

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
        try:
            pose = str(poseName.getName())
            if pose == 'carry':
                return self.spot.spot_arm.arm_carry()
            if pose == 'detection':
#                 return self.spot.spot_arm.hand_pose(x=0.818, y=-0.003, z=0.523, qx=-0.000, qy=0.389, qz=-0.004, qw=0.921)
                return self.spot.spot_arm.hand_pose(x=0.6, y=-0.001, z=0.3, qx=0.003, qy=0.233, qz=0.001, qw=0.973)
            if pose == "putdown" or pose == "putDown":
                return self.spot.spot_arm.hand_pose(x=1.1, y=0, z=0.4, qx=-0.7071, qy=0, qz=0, qw=0.7071)
            if pose == 'stow':
                # allow "stow" arm position while holding an object
                self.spot.spot_arm.override_grasp_or_carry(
                    grasp_override=manipulation_api_pb2.ApiGraspOverride.Override.OVERRIDE_UNKNOWN,
                    carry_override=ManipulatorState.CarryState.CARRY_STATE_CARRIABLE_AND_STOWABLE)

                return self.spot.spot_arm.arm_stow()
            if pose == 'unstow':
                return self.spot.spot_arm.arm_unstow()
        except Exception as e:
            logging.error(f"Error calling goToPose1: {traceback.format_exc()}")
            return False, "error msg"

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

        try:
            if len(args)==1:
                return self.goToLocationRef(args[0])
            if len(args)==2 and isinstance(args[0], Symbol) and isinstance(args[1], JBoolean):
                return self.goToLocationRefWait(*args)
            if len(args)==2 and isinstance(args[0], Symbol) and isinstance(args[1], Symbol):
                return self.goToLocationFrom(*args)
            if len(args)==7:
                return self.goToLocationPose(*args)
        except Exception as e:
            logging.error(f"Error calling goToLocation: {traceback.format_exc()}")
            return (False, "error msg")

        logging.info(f"No valid argument match found.")
        raise RuntimeError("Incorrect arguments")

    def goToLocationRef(self, location):
        ref = self.locationConsultant.getRefWithId(location)
        if ref is None:
            return (False, f"No reference found with name {location}")

        waypoint_id = ref.getWaypointId()
        return self.spot.spot_graph_nav.navigate_to_existing_waypoint(str(waypoint_id))

    def goToLocationRefWait(self, location, wait):
        pass

    def goToLocationFrom(self, desiredLocation, initialLocation):
        # todo: get ininital loc
        return self.goToLocation(desiredLocation)

    def goToLocationPose(self, xdest, ydest, quat_x, quat_y, quat_z, quat_w, wait):
        pass

    #####################################################################

    @handle_return
    @JOverride
    def approachLocation(self, refId):

        # TODO: choose ray vs pixel based on contents in MO (i.e., if mo.getLocation() has nan values)
        walk_to_object_in_image = True
        if walk_to_object_in_image:
            # use 2D pixel based bounding box to specify object to grasp -- assumes spot hand camera
            image_response = self.spot.spot_images.get_hand_rgb_image()
            (x_pix, y_pix) = self._get_center_pixel(refId, image_response.source.cols, image_response.source.rows)
            walk = manipulation_api_pb2.WalkToObjectInImage(
                pixel_xy=Vec2(x=x_pix, y=y_pix),
                transforms_snapshot_for_camera=image_response.shot.transforms_snapshot,
                frame_name_image_sensor=image_response.shot.frame_name_image_sensor,
                camera_model=image_response.source.pinhole,
            )
        else:
            # use ray to specify object to grasp -- requires 3D camera data
            ret = self.trade.call_trade("getEntityForReference", [], refId, JClass("edu.tufts.hrilab.vision.stm.MemoryObject"))
            mo = JClass("edu.tufts.hrilab.vision.stm.MemoryObject")(ret)

            mo.transformToBase()
            start = Point3d(0, 0, 0)
            mo.getBaseTransform().transform(start)

            end = Point3d(start.x, start.y, start.z)
            direction = mo.getDirection()
            direction.scale(4.0)
            end.add(direction)
            walk = manipulation_api_pb2.WalkToObjectRayInWorld(
                ray_start_rt_frame=geometry_pb2.Vec3(x=start.x, y=start.y, z=start.z),
                ray_end_rt_frame=geometry_pb2.Vec3(x=end.x, y=end.y, z=end.z),
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
        try:
            return self.spot.stop()
        except Exception as e:
            logging.error(f"Error calling stop: {traceback.format_exc()}")
            return False, ""

    @handle_return
    @JOverride
    def isMoving(self):
        isMoving = self.spot.is_moving
        logging.info(f"isMoving = {isMoving}")
        return isMoving, ""

    @JOverride
    def checkAt(self, locationTerm):
        # TODO: is there a way to actually check if we are at a location?
        # return "True" observation
        from java.util import HashMap, ArrayList
        java_list = ArrayList()
        java_list.add(HashMap())  # Add an empty HashMap to the list
        return java_list

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
    def grabObject(self, refId):
        try:

            # TODO: choose ray vs pixel based on contents in MO (i.e., if mo.getLocation() has nan values)
            pick_object_in_image = True
            if pick_object_in_image:
                # use 2D pixel based bounding box to specify object to grasp -- assumes spot hand camera
                image_response = self.spot.spot_images.get_hand_rgb_image()
                (x_pix, y_pix) = self._get_center_pixel(refId, image_response.source.cols, image_response.source.rows)
                logging.info(f"Pick up object at pixel: {x_pix}, {y_pix}")

                grasp = manipulation_api_pb2.PickObjectInImage(
                    pixel_xy=Vec2(x=x_pix, y=y_pix),
                    transforms_snapshot_for_camera=image_response.shot.transforms_snapshot,
                    frame_name_image_sensor=image_response.shot.frame_name_image_sensor,
                    camera_model=image_response.source.pinhole,
                    walk_gaze_mode=2
                )
            else:
                # use ray to specify object to grasp -- requires 3D camera data
                # NOT FULLY TESTED
                ret = self.trade.call_trade("getEntityForReference", [], refId, JClass("edu.tufts.hrilab.vision.stm.MemoryObject"))
                mo = JClass("edu.tufts.hrilab.vision.stm.MemoryObject")(ret)

                mo.transformToBase()
                start = Point3d(0, 0, 0)
                mo.getBaseTransform().transform(start)
                # logging.info(f"MO transform {mo.getBaseTransform()}")

                end = Point3d(start.x, start.y, start.z)
                direction = mo.getDirection()
                direction.scale(4.0)
                end.add(direction)
                # logging.info(f"Pick up object at ray. Start: {start.x}, {start.y}, {start.z} End: {end.x}, {end.y}, {end.z}")

                grasp = manipulation_api_pb2.PickObjectRayInWorld(
                    ray_start_rt_frame=geometry_pb2.Vec3(x=start.x, y=start.y, z=start.z),
                    ray_end_rt_frame=geometry_pb2.Vec3(x=end.x, y=end.y, z=end.z),
                    frame_name=self._reference_frame,
                    walk_gaze_mode=2
                )


            # hint about camera source used (2 == hand camera)
            grasp.grasp_params.manipulation_camera_source = 2

            # setting grasp params restricts the types of objects that can be picked up (top-down vs side, etc),
            # so it doesn't make for a very general grabObject method
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

                # Frame name for the frame that the constraints in allowable_orientation are expressed in
                grasp.grasp_params.grasp_params_frame_name = GROUND_PLANE_FRAME_NAME

                # We'll take anything within about 15 degrees for top-down or horizontal grasps.
                constraint.vector_alignment_with_tolerance.threshold_radians = 0.5

            # Build the proto
            if pick_object_in_image:
                grasp_request = manipulation_api_pb2.ManipulationApiRequest(
                    pick_object_in_image=grasp)
            else:
                grasp_request = manipulation_api_pb2.ManipulationApiRequest(
                    pick_object_ray_in_world=grasp)

            (success, msg, cmd_id) = self.spot.manipulation_command(grasp_request)
            self.spot.spot_arm.block_until_manipulation_completes(self.spot._manipulation_api_client, cmd_id)
            logging.info(f"Pickup command results: {success}, {msg} {cmd_id}")
            return success, msg, cmd_id

        except Exception as e:
            logging.error(f"Error calling grabObject: {traceback.format_exc()}")
            return False, "error msg"

    @JOverride
    def isHoldingObject(self) -> bool:
        is_holding = self.spot._robot_state_client.get_robot_state().manipulator_state.is_gripper_holding_item
        logging.info(f"Is Spot holding an object: {is_holding}")
        return is_holding

    @handle_return
    @JOverride
    def localizeWithReference(self, refId):
        try:
            ref = self.locationConsultant.getRefWithId(refId)
            if ref is None:
                return (False, f"Cannot localize. No reference found for reference: {refId}")

            waypoint_id = str(ref.getWaypointId())
            self.spot.spot_graph_nav.set_initial_localization_waypoint([waypoint_id])
            return True, ""
        except Exception as e:
            logging.error(f"Error calling localizeWithReference: {traceback.format_exc()}")
            return False, ""

    @handle_return
    @JOverride
    def localizeWithFiducial(self):
        try:
            self.spot.spot_graph_nav.set_initial_localization_fiducial()
            return True, ""
        except Exception as e:
            logging.error(f"Error calling localizeWithFiducial: {traceback.format_exc()}")
            return False, ""

    @JOverride
    def isLocalized(self) -> bool:
        try:
            return not self.spot.spot_graph_nav._graph_nav_client.get_localization_state().lost_detector_state.is_lost
        except Exception as e:
            logging.error(f"Error calling isLocalized: {traceback.format_exc()}")
            return False

    @JOverride
    def startMapping(self):
        """
        Puts the system in recording mode, which will allow the create of new waypoints with create_waypoint(...)
        :return:
        """

        try:
            if self._mapping:
                logging.info("Already mapping")
                return
            self._recording_client = self.spot._robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
            self._recording_client.start_recording()
            self._mapping = True
        except Exception as e:
            logging.error(f"Error starting recording: {traceback.format_exc()}")
            return False, "Error starting recording"

    @JOverride
    def createWaypoint(self, name: Symbol):
        """
        Creates a waypoint at the robot's current location. Requires start_mapping(...) to have been called.
        Note that this doesn't merge this waypoint until finish_mapping(...) is called.
        :param name: Human-readable name of the waypoint
        :return:
        """
        try:
            if not self._mapping:
                logging.warning("Not mapping. First call startMapping.")
                return
            record_resp = self._recording_client.create_waypoint()
            self.locationConsultant.createReferenceWithWaypointId(JString(record_resp.created_waypoint.id), name)
            logging.info(f"New waypoint ID: {record_resp.created_waypoint.id} with name: {name}")
        except Exception as e:
            logging.error(f"Error creating waypoint: {traceback.format_exc()}")
            return False, "Error creating waypoint"

    @JOverride
    def finishMapping(self):
        """
        Turns off recording mode and merges any new waypoints into the current graph.
        TODO: There may be some anchor optimization that could happen here. If you see weird nav behaviors, I would check that out first.
        :return:
        """
        try:
            if not self._mapping:
                logging.warning("Not mapping. First call startMapping.")
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

            self.locationConsultant.writeReferencesToFile("/tmp/spot_map_refs.json")

        except Exception as e:
            logging.error(f"Error finishing mapping: {traceback.format_exc()}")
            return False, "Error finishing mapping"

    @JOverride
    @handle_return
    def getTV(self):
        pass

    @JOverride
    @handle_return
    def getRV(self):
        pass

    @JOverride
    @handle_return
    def getVels(self):
        pass

    @JOverride
    @handle_return
    def getDefaultVels(self):
        pass

    @JOverride
    def setTV(self, tv):
        try:
            logging.info(f"Setting TV({tv})")
            return self._setVelsLocal(tv, 0.0, 0.0)
        except Exception as e:
            logging.error(f"Error calling setTV({tv}): {traceback.format_exc()}")
            return False

    @JOverride
    def setRV(self, rv):
        try:
            logging.info(f"Setting RV({rv})")
            return self._setVelsLocal(0.0, 0.0, rv)
        except Exception as e:
            logging.error(f"Error calling setRV vel({rv}): {traceback.format_exc()}")
            return False

    @JOverride
    def setVels(self, *args):
        logging.info(f"setVels dispatch method called")
        # always use the wild card args when implementing a dispatch

        try:
            if len(args)==2:
                return self._setVelsLocal(args[0], 0.0, args[1])
            if len(args)==3:
                return self._setVelsLocal(args[0], args[1], args[2])

        except Exception as e:
            logging.error(f"Error calling setVels: {traceback.format_exc()}")
            return False

        logging.info(f"No valid argument match found.")
        raise RuntimeError("Incorrect arguments")

    def _setVelsLocal(self, vel_x, vel_y, vel_rot):
        try:
            # Build the full mobility command
            velocity_command = RobotCommandBuilder.synchro_velocity_command(
                v_x=float(vel_x),
                v_y=float(vel_y),
                v_rot=float(vel_rot)
            )

            (success, msg, cmd_id) = self.spot.robot_command(velocity_command, duration=2)
            return success

        except Exception as e:
            logging.error(f"Error calling setVels: {traceback.format_exc()}")
            return False


# This main is largely used for testing behaviors
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Spot TRADE Wrapper')
    parser.add_argument('--spot-config', required=True, help='Path to Spot yaml config file')
    parser.add_argument('--spot-map', help='Path to Spot graph_nav file')
    parser.add_argument('--refs', help='Path to Spor refs file')
    parser.add_argument('--groups', nargs='*', help='TRADE group(s)')
    parser.add_argument('--test', action='store_true', help='run test code in main method')
    args = parser.parse_args()

    with open(args.spot_config, 'r') as file:
        config = yaml.safe_load(file)
        config = config['/**']['ros__parameters']

    trade_wrapper = TRADEWrapper()

    spot_wrapper = SpotTRADEWrapper(trade_wrapper, config, args.refs)
    logging.info(f"TRADE groups: {args.groups}")
    TRADE.registerAllServices(spot_wrapper, args.groups if args.groups else "") # TODO: fix the args.groups when more than one group is specified
    TRADE.registerAllServices(spot_wrapper.locationConsultant, "location")

    spot_wrapper.claim()

    # claim robot
    if args.test:
        spot_wrapper.spot.power_on()
        spot_wrapper.spot.spot_arm.gripper_open()

    # load pre-built spot map
    if args.spot_map:
        logging.info(f"Loading spot map: {args.spot_map}")
        (result, msg) = spot_wrapper.spot.spot_graph_nav.upload_graph(args.spot_map)
        if not result:
            logging.warning(f"Failed to load spot map: {args.spot_map}. Message: {msg}")

        # spot_wrapper.undock() # TODO: is undock-ing needed here?
        pts = spot_wrapper.spot.spot_graph_nav.list_graph()  # Have to call this to update map
        # logging.info(spot_wrapper.spot.spot_graph_nav._current_annotation_name_to_wp_id["test_two"])
