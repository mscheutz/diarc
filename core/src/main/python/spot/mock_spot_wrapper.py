import typing

from bosdyn.api import manipulation_api_pb2, geometry_pb2
from bosdyn.client import frame_helpers
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.math_helpers import Vec3
from jpype import JImplements, JOverride, JClass
from spot_wrapper.wrapper import SpotWrapper
from pyTRADE.core.wrapper import TRADEWrapper

from edu.tufts.hrilab.interfaces import DockingInterface, MoveBaseInterface, ArmInterface
from edu.tufts.hrilab.spot import SpotInterface
from javax.vecmath import Point3d, Matrix4d, Vector3d, Quat4d
from edu.tufts.hrilab.action.justification import ConditionJustification

@JImplements([DockingInterface, MoveBaseInterface, ArmInterface, SpotInterface])
class MockSpotTRADEWrapper:
    def __init__(self, config, trade_wrapper: TRADEWrapper):
        self._trade_wrapper = trade_wrapper
        self._reference_frame = "gpe"

    @JOverride
    def moveTo(self, groupName, point, orientation):
        pass

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
    def stopRecordingTrajectory(self, ):
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

    @JOverride
    def closeGripper(self, groupName):
        pass

    @JOverride
    def openGripper(self, groupName):
        pass

    @JOverride
    def closeGripper(self):
        return self.closeGripper("")

    @JOverride
    def openGripper(self):
        return self.openGripper("")

    @JOverride
    def moveGripper(self, groupName, meters):
        pass

    @JOverride
    def getGripperPosition(self, groupName):
        pass

    @JOverride
    def pressObject(self, group_name, object_location, object_orientation):
        pass

    @JOverride
    def pressObject(self, group_name, refID):
        pass

    @JOverride
    def dock(self, dockId):
        # Todo: Symbol -> id
        pass

    @JOverride
    def undock(self):
        pass

    @JOverride
    def getPoseGlobalQuat(self):
        pass

    @JOverride
    def setPoseGlobal(self, x, y, theta):
        pass

    @JOverride
    def goToLocation(self, location, wait):
        pass

    @JOverride
    def goToLocation(self, location):
        pass
        # return self.spot_wrapper.spot_graph_nav.navigate_to_existing_waypoint()

    @JOverride
    def goToLocation(self, desiredLocation, initialLocation):
        pass

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
    def stop(self):
        pass

    @JOverride
    def isMoving(self):
        return self._spot_wrapper.is_moving

    @JOverride
    def checkAt(self, locationTerm):
        pass

    @JOverride
    def command(self, input):
        pass

    @JOverride
    def claim(self):
        pass

    @JOverride
    def release(self):
        pass

    @JOverride
    def stand(self):
        pass

    @JOverride
    def sit(self):
        pass

    @JOverride
    def powerOn(self):
        pass

    @JOverride
    def powerOff(self):
        pass

    @JOverride
    def stow(self):
        pass

    @JOverride
    def unstow(self):
        pass

    @JOverride
    def carry(self):
        pass

    @JOverride
    def goToStartPose(self, safe):
        pass

    @JOverride
    def getTransform(self, dstFrame):
        pass
    @JOverride
    def getTransform(self, srcFrame, dstFrame):
        pass

    def _handle_return(self, value: typing.Tuple[bool, str]):
        return ConditionJustification(value[0])

    @JOverride
    def pickUpObject(self, refId):
        source_frame = "hand_color_image_sensor"

        mo = self._trade_wrapper.call_trade("getEntityForReference", refId,
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

        pass