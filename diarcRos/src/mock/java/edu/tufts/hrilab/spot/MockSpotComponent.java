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
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.List;

public class MockSpotComponent extends DiarcComponent implements ArmInterface {
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

    @Action
    @TRADEService
    public Justification dock(Symbol dockId) {
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification undock() {
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public boolean detectAndOpenDoor() {
        return true;
    }

    @Action
    @TRADEService
    public boolean openDoor() {
        return true;
    }

    @Action
    @TRADEService
    public boolean stand() {
        return true;
    }

    @Action
    @TRADEService
    public boolean sit() {
        return true;
    }

    @Action
    @TRADEService
    public Justification moveArmOverTable() {
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification moveArmOverBody() {
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification stowArm() {
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification unStowArm() {
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification moveArmToCarryPosition() {
        log.info("[moveArmToCarryPosition]");
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            log.error("moveArmToCarryPosition",e);
        }
        return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification openGripper() {
        log.info("[openGripper]");
        return new ConditionJustification(true);
    }
    @Action
    @TRADEService
    public Justification closeGripper() {
        return new ConditionJustification(true);
    }

    @Override
    public Justification moveTo(String groupName, Point3d point, Quat4d orientation) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveTo(String groupName, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveTo(String groupName, Symbol refId) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveToRelative(String groupName, Point3d point, Quat4d orientation) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification graspObject(String groupName, Symbol refId, float position) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification releaseObject(String groupName, Symbol refId, float position) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification pointTo(String groupName, Symbol objectRef) {
        return new ConditionJustification(false);
    }

    @Override
    public Pair<Point3d, Quat4d> getEEPose(String groupName) {
        return null;
    }

    @Override
    public Pair<Point3d, Quat4d> getPose(String linkName) {
        return null;
    }

    @Override
    public Justification recordPose(Symbol poseName) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification saveEEPosesToFile(String filename) {
        return new ConditionJustification(false);
    }

    @Override
    public void loadEEPosesFromFile(String filename) {
    }

    @Override
    public Justification savePosesToFile(String filename) {
        return new ConditionJustification(false);
    }

    @Override
    public void loadPosesFromFile(String filename) {

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
    }

    @Override
    public void stopRecordingTrajectory() {
    }

    @Override
    public Justification executeTrajectory(String trajectoryName) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification saveTrajectoriesToFile(String filename) {
        return new ConditionJustification(false);
    }

    @Override
    public void loadTrajectoriesFromFile(String filename) {
    }

    @Override
    public Justification closeGripper(String groupName) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification openGripper(String groupName) {
        return new ConditionJustification(false);
    }

    @Override
    public Justification moveGripper(String groupName, float meters) {
        return new ConditionJustification(false);
    }

    @Override
    public float getGripperPosition(String groupName) {
        return 0;
    }

    @Override
    public Justification pressObject(String group_name, Point3d object_location, Quat4d object_orientation){
        return new ConditionJustification(true);
    }

    @Override
    public Justification pressObject(String group_name, Symbol refID){
        return new ConditionJustification(true);
    }
}
