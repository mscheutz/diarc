/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.interfaces.ArmInterface;
import edu.tufts.hrilab.interfaces.DockingInterface;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist;
import edu.tufts.hrilab.diarcros.msg.spot_msgs.ArmJointMovementRequest;
import edu.tufts.hrilab.diarcros.msg.spot_msgs.ArmJointMovementResponse;
import edu.tufts.hrilab.diarcros.msg.spot_msgs.DockRequest;
import edu.tufts.hrilab.diarcros.msg.spot_msgs.DockResponse;
//import edu.tufts.hrilab.diarcros.msg.spot_msgs.SpotPoseRequest;
//import edu.tufts.hrilab.diarcros.msg.spot_msgs.SpotPoseResponse;
import edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerRequest;
import edu.tufts.hrilab.diarcros.msg.std_srvs.TriggerResponse;
import edu.tufts.hrilab.diarcros.spot_ros.SpotDoorDetector;
import edu.tufts.hrilab.diarcros.spot_ros.SpotRos;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.List;

public class SpotComponent extends DiarcComponent implements ArmInterface, DockingInterface {
    SpotRos spotRos;
    private SpotDoorDetector spotDoorDetector;
    Twist cmd_vel;
    boolean do_async_pub;
    //Thread spinner;

    // Regular class setup stuff
    public SpotComponent() {
        super();
        cmd_vel = new Twist();
        do_async_pub = false;
    }

    @Override
    public void init() {
        super.init();
        spotRos = new SpotRos();
        spotDoorDetector = new SpotDoorDetector();
    }

    @Action
    @TRADEService
    public boolean walkForward() {
        //cmdVelPublisher.sendCmdVel(cmdVelPublisher.generateLinearCommand(0.75));
        return true;
    }

    @Action
    @TRADEService
    public boolean walkBackward() {
        //cmdVelPublisher.sendCmdVel(cmdVelPublisher.generateLinearCommand(-0.75));
        return true;
    }

    @Action
    @TRADEService
    public boolean turnRight() {
        //cmdVelPublisher.sendCmdVel(cmdVelPublisher.generateAngularCommand(-1));
        return true;
    }

    @Action
    @TRADEService
    public boolean turnLeft() {
        //cmdVelPublisher.sendCmdVel(cmdVelPublisher.generateAngularCommand(1));
        return true;
    }

    @Action
    @TRADEService
    public boolean stopWalking() {
        //cmdVelPublisher.sendStopCmdVel();
        return true;
    }

//    @Action
//    @TRADEService
//    public boolean lookUp() {
//        byte LOOK_UP = 3;
//        SpotPoseResponse resp = new SpotPoseResponse();
//        spotRos.callSpotSpotPose(new SpotPoseRequest(LOOK_UP, 0,0,0,0,0), resp);
//        Util.Sleep(1500);
//        stand();
//        return resp.getSuccess();
//    }

//    @Action
//    @TRADEService
//    public boolean lookDown() {
//        byte LOOK_DOWN = 4;
//        SpotPoseResponse resp = new SpotPoseResponse();
//        spotRos.callSpotSpotPose(new SpotPoseRequest(LOOK_DOWN, 0,0,0,0,0), resp);
//        Util.Sleep(1500);
//        stand();
//        return resp.getSuccess();
//    }

//    @Action
//    @TRADEService
//    public boolean lookLeft() {
//        byte CUSTOM = 6;
//        SpotPoseResponse resp = new SpotPoseResponse();
//        spotRos.callSpotSpotPose(new SpotPoseRequest(CUSTOM, 0.3,0,0,0,0), resp);
//        Util.Sleep(1500);
//        stand();
//        return resp.getSuccess();
//    }

//    @Action
//    @TRADEService
//    public boolean lookRight() {
//        byte CUSTOM = 6;
//        SpotPoseResponse resp = new SpotPoseResponse();
//        spotRos.callSpotSpotPose(new SpotPoseRequest(CUSTOM, -0.3,0,0,0,0), resp);
//        Util.Sleep(1500);
//        stand();
//        return resp.getSuccess();
//    }

    // Spot specific actions

    @Action
    @TRADEService
    public Justification dock(Symbol dockId) {
        stopWalking();
        //Justification complete = goTo(knownLocations.get("dock"), true);
        //if (!complete.getValue())
        //    return false;
        DockResponse response = new DockResponse();
        int idAsInt = Integer.parseInt(dockId.getName());
        log.error("hacky code! Dock id is hard-coded to 521 because that's how the demo is set up. It should be taking in a symbol so other docks can be used!");
        idAsInt = 521;
        spotRos.callSpotDock(new DockRequest(idAsInt), response);
        return new ConditionJustification(response.getSuccess());
    }

    @Action
    @TRADEService
    public boolean openDoor() {
        stopWalking();
        return spotRos.callSpotOpenDoor();
    }

    @Action
    @TRADEService
    public Justification undock() {
        return new ConditionJustification(spotRos.callSpotUndock());
    }

    @Action
    @TRADEService
    public boolean stand() {
        return spotRos.callSpotStand();
    }

    @Action
    @TRADEService
    public boolean sit() {
        stopWalking();
        Util.Sleep(100);
        return spotRos.callSpotSit();
    }

    @Action
    @TRADEService
    public Justification moveArmOverTable() {
        //double[] jointPositions = new double[]{-0.9807476997375488, -0.7576398849487305, 0.7608270645141602, 0.020904064178466797, 1.5774314403533936, 0.017466306686401367};
        //double[] jointPositions = new double[]{-0.9470815658569336, -0.9033534526824951, 0.9177565574645996, -0.1418623924255371, 1.6009159088134766, 0.7565438747406006};
        double[] jointPositions = new double[]{-1.5687861442565918, -0.8387935161590576, 0.775935173034668, -0.04880070686340332, 1.6157169342041016, 1.5050809383392334};
        ArmJointMovementResponse response = new ArmJointMovementResponse();
        spotRos.callSpotArmJointMove(new ArmJointMovementRequest(jointPositions), response);
        return new ConditionJustification(response.getSuccess());
    }

    @Action
    @TRADEService
    public Justification detectAndOpenDoor() {
        TriggerResponse response = new TriggerResponse();
        spotDoorDetector.callSpotRunOpenDoor(new TriggerRequest(), response);
        if (response.getSuccess()) {
            log.info("[detectAndOpenDoor] door opening successful");
        } else {
            log.error("[detectAndOpenDoor] failed to find and/or open door. retrying once."); //todo: this doesn't block, and the timeout is unsophisticated. fix in ros.
            spotDoorDetector.callSpotRunOpenDoor(new TriggerRequest(), response);
            if (response.getSuccess()) {
                log.info("[detectAndOpenDoor] door opening successful");
            } else {
                log.error("[detectAndOpenDoor] failed to find and/or open door. exiting.");
            }
        }
        return new ConditionJustification(response.getSuccess());
    }

    @Action
    @TRADEService
    public Justification moveArmOverBody() {
        double[] jointPositions = new double[]{-0.05496811866760254, -2.8640613555908203, 2.4309966564178467, -0.021846532821655273, 0.48256170749664307, 0.0053408145904541016};
        ArmJointMovementResponse response = new ArmJointMovementResponse();
        spotRos.callSpotArmJointMove(new ArmJointMovementRequest(jointPositions), response);
        return new ConditionJustification(response.getSuccess());
    }

    @Action
    @TRADEService
    public Justification stowArm() {
        TriggerResponse response = new TriggerResponse();
        spotRos.callSpotArmStow(new TriggerRequest(),response);
        return new ConditionJustification(response.getSuccess());
    }

    @Action
    @TRADEService
    public Justification unStowArm() {
        TriggerResponse response = new TriggerResponse();
        spotRos.callSpotArmUnstow(new TriggerRequest(),response);
        return new ConditionJustification(response.getSuccess());
    }

    @Action
    @TRADEService
    public Justification moveArmToCarryPosition() {
        TriggerResponse response = new TriggerResponse();
        spotRos.callSpotArmCarry(new TriggerRequest(),response);
        return new ConditionJustification(response.getSuccess());
    }

    @Action
    @TRADEService
    public Justification openGripper() {
        TriggerResponse response = new TriggerResponse();
        spotRos.callSpotGripperOpen(new TriggerRequest(),response);
        return new ConditionJustification(response.getSuccess());
    }
    @Action
    @TRADEService
    public Justification closeGripper() {
        TriggerResponse response = new TriggerResponse();
        spotRos.callSpotGripperClose(new TriggerRequest(),response);
        return new ConditionJustification(response.getSuccess());
    }

    // ArmComponent interface implementation
    @Override
    public Justification moveTo(String groupName, Point3d point, Quat4d orientation) {
        log.error("moveTo not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveTo(String groupName, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
        log.error("moveTo not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveTo(String groupName, Symbol refId) {
        log.error("moveTo not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
        log.error("moveTo not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveToRelative(String groupName, Point3d point, Quat4d orientation) {
        log.error("moveToRelative not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification graspObject(String groupName, Symbol refId, float position) {
        log.error("graspObject not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification releaseObject(String groupName, Symbol refId, float position) {
        log.error("releaseObject not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification pointTo(String groupName, Symbol objectRef) {
        log.error("pointTo not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Pair<Point3d, Quat4d> getEEPose(String groupName) {
        log.error("getEEPose not implemented");
        return null;
    }

    @Override
    public Pair<Point3d, Quat4d> getPose(String linkName) {
        log.error("getPose not implemented");
        return null;
    }

    @Override
    public Justification recordPose(Symbol poseName) {
        log.error("recordPoses not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification savePosesToFile(String filename) {
        log.error("savePosesToFile not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public void loadPosesFromFile(String filename) {
        log.error("loadPosesFromFile not implemented");
    }

    @Override
    public Justification goToPose(String groupName, Symbol poseName) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification goToPose(Symbol poseName) {
        return new ConditionJustification(false);
    }

    @Override
    public void startRecordingTrajectory(String trajectoryName) {
        log.error("startRecordingTrajectory not implemented");
    }

    @Override
    public void stopRecordingTrajectory() {
        log.error("stopRecordingTrajectory not implemented");
    }

    @Override
    public Justification executeTrajectory(String trajectoryName) {
        log.error("executeTrajectory not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification saveTrajectoriesToFile(String filename) {
        log.error("saveTrajectoriesToFile not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public void loadTrajectoriesFromFile(String filename) {
        log.error("loadTrajectoriesFromFile not implemented");
    }

    @Override
    public Justification closeGripper(String groupName) {
        log.error("closeGripper not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification openGripper(String groupName) {
        log.error("openGripper not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveGripper(String groupName, float meters) {
        log.error("moveGripper not implemented");
        return new ConditionJustification(false);
    }

    @Override
    public float getGripperPosition(String groupName) {
        log.error("getGripperPosition not implemented");
        return 0;
    }

}
