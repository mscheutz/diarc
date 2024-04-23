/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.arm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import org.apache.commons.cli.Option;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;

public class MockArmComponent extends DiarcComponent implements MockArmInterface {

    //TODO:brad: do we actually need this?
    private Map<Symbol,Pair<Point3d,Quat4d>>storedPoses = new HashMap<>();

    PoseConsultant consultant;

    private long sleepDuration;
    @Override
    protected void init() {
        sleepDuration = 3000;
        consultant = new PoseConsultant(PoseReference.class, "pose", new ArrayList<>());
        try {
            Collection<String> groups = this.getMyGroups();
            groups.add(consultant.getKBName());
            TRADE.registerAllServices(consultant,groups);
        } catch (TRADEException e) {
            log.error("Error registering with trade ", e);
        }
    }

    public MockArmComponent() {
        super();
    }

    @Override
    public boolean moveTo(String groupName, Point3d point, Quat4d orientation) {
        log.info("[moveTo] " + groupName + " " + point + " " + orientation);
        return true;
    }

    @Override
    public boolean moveTo(String groupName, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
        return false;
    }

    public boolean moveTo(Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
        String nameofCurrMethod = new Throwable().getStackTrace()[0].getMethodName();
        log.info("Entered method" + nameofCurrMethod);
        return true;
    }

    @Override
    public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
        return new ConditionJustification(true);
    }

    @Override
    public Justification moveTo(String group_name, Symbol objectRef) {
        return new ConditionJustification(true);
    }

    @Override
    public boolean pointTo(String groupName, Symbol objectRef) {
        log.info("[pointTo] " + groupName + " " + objectRef);
        getMemoryObject(objectRef);
        Util.Sleep(sleepDuration);

        return true;
    }

    @Override
    public Justification moveToRelative(String group_name, Point3d point, Quat4d orientation) {
        String nameofCurrMethod = new Throwable().getStackTrace()[0].getMethodName();
        log.info("Entered method" + nameofCurrMethod);
        return new ConditionJustification(true);
    }

    @Override
    public Justification graspObject(String groupName, Symbol refId, float position) {
        return new ConditionJustification(true);
    }

    @Override
    public Justification releaseObject(String groupName, Symbol refId, float position) {
        return null;
    }

    @Override
    public Pair<Point3d, Quat4d> getEEPose(String groupName) {
        log.info("[getEEPose] " + groupName);
        return null;
    }

    @Override
    public Pair<Point3d, Quat4d> getPose(String linkName) {
        log.info("[getPose] " + linkName);
        return null;
    }

    @Override
    public boolean recordPose(Symbol poseName) {

        //"get" current pose
        Pair<Point3d, Quat4d> pose = Pair.of(new Point3d(), new Quat4d());
        //record current pose
        storedPoses.put(poseName,pose);

        //add pose name to properties consultant can handle
        List<Term> props = new ArrayList<>();
        props.add(Factory.createPredicate(poseName.getName(), "X"));
        consultant.addPropertiesHandled(props);

        //Get a new refId from consultant
        List<Variable> vars = new ArrayList<>();
        Variable var = Factory.createVariable("X");
        vars.add(var);
        Map<Variable, Symbol> refIds = consultant.createReferences(vars);

        //Notify consultant we have an instance of new location type
        consultant.assertProperties(refIds.get(var), props);

        //add pose info to created reference
        PoseReference poseReference= consultant.getReference(refIds.get(var));
        poseReference.setPose(pose.getLeft(),pose.getRight());

        //inform parser of new location
        log.debug("adding pose " + poseName + " to parser dict");
        //TODO reimplement homophone permutations
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("injectDictionaryEntry")).call(void.class, poseName.getName(), "POSE", poseName.getName(), "DEFINITE");
        } catch (TRADEException e) {
            log.error("unable to add dictionary entry for " + poseName, e);
        }

        return true;
    }

    @Override
    public boolean saveEEPosesToFile(String filename) {
        return false;
    }

    @Override
    public void loadEEPosesFromFile(String filename) {

    }

    @Override
    public boolean savePosesToFile(String filename) {
        log.info("[savePosesToFile] " + filename);
        return true;
    }

    @Override
    public void loadPosesFromFile(String filename) {
        log.info("[loadPosesFromFile] " + filename);
    }

    @Override
    public boolean goToPose(Symbol poseName) {
        if(storedPoses.containsKey(poseName)) {
            log.info("[goToPose] " + poseName);
            Util.Sleep(sleepDuration);
            return true;
        }else{
            log.error("[goToPose] trying to go to unknown pose: "+poseName);
            return false;
        }
    }

    @Override
    public boolean goToPose(String poseName, Symbol groupName) {
        log.info("[goToPose] " + poseName + " " + groupName);
        Util.Sleep(sleepDuration);

        return true;
    }

    @Override
    public boolean goToStartPose(boolean safe) {
        log.info("[goToStartPose] " + safe);
        Util.Sleep(sleepDuration);

        return true;
    }

    @Override
    public void startRecordingTrajectory(String trajectory_name) {

    }

    @Override
    public void stopRecordingTrajectory() {

    }

    @Override
    public boolean executeTrajectory(String trajectory_name) {
        return true;
    }

    @Override
    public boolean saveTrajectoriesToFile(String filename) {
        return true;
    }

    @Override
    public void loadTrajectoriesFromFile(String filename) {

    }

    @Override
    public Justification closeGripper(String groupName) {
        String nameofCurrMethod = new Throwable().getStackTrace()[0].getMethodName();
        log.info("Entered method" + nameofCurrMethod);
        return new ConditionJustification(true);
    }

    @Override
    public Justification openGripper(String groupName) {
        String nameofCurrMethod = new Throwable().getStackTrace()[0].getMethodName();
        log.info("Entered method" + nameofCurrMethod);
        return new ConditionJustification(true);
    }

    @Override
    public boolean moveGripper(String groupName, float position) {
        Util.Sleep(sleepDuration);
        return true;
    }

    @Override
    public float getGripperPosition(String groupName) {
        return 0;
    }

    @Override
    public Justification pressObject(String group_name, Point3d object_location, Quat4d object_orientation) {
        return null;
    }

    @Override
    public Justification pressObject(String group_name, Symbol refID) {
        return null;
    }

    @Override
    public boolean startPouring(double ang) {
        log.info("Start Pouring, angle: " + ang);
        Util.Sleep(sleepDuration);

        return true;
    }

    @Override
    public boolean startPouring(Symbol obj) {
        log.info("Start Pouring, object: " + obj);
        Util.Sleep(sleepDuration);

        return true;
    }

    @Override
    public boolean stopPouring(Symbol obj) {
        log.info("Stop Pouring: " + obj);
        return true;
    }

    @Override
    public boolean stopPouring() {
        log.info("Stop Pouring");
        return true;
    }

    @Override
    public boolean moveAbove(Symbol s, Symbol s2) {
        log.info("Moving " + s + " above " + s2);
        Util.Sleep(sleepDuration);

        return true;
    }


    @Override
    protected List<Option> additionalUsageInfo() {
        return new ArrayList<>();
    }

    protected MemoryObject getMemoryObject(Symbol objectRef) {
        List<MemoryObject> mos = null;
        log.debug("getting Memory Object for " + objectRef);
        try {
            TRADEServiceConstraints additionalConstraints = new TRADEServiceConstraints();
            if (!getMyGroups().isEmpty()) {
                additionalConstraints.inGroups(getMyGroups().toArray(new String[0]));
            }

            mos = TRADE.getAvailableService(additionalConstraints.name("getTokens")).call(List.class, objectRef);

            if (mos == null || mos.isEmpty()) {
                log.warn("Could not find MemoryObject with object ref: " + objectRef);
                return null;
            }
        } catch (TRADEException e) {
            log.error("[getMemoryObject] Exception while trying to call getTokens. Returning null.", e);
        }

        // just choosing the first one for now
        MemoryObject mo = mos.get(0);
        log.info("got Memory Object: " + mo);
        return mo;
    }

    public static Quat4d convertRPY(double roll, double pitch, double yaw) {
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double w = (cy * cp * cr + sy * sp * sr);
        double x = (cy * cp * sr - sy * sp * cr);
        double y = (sy * cp * sr + cy * sp * cr);
        double z = (sy * cp * cr - cy * sp * sr);

        return new Quat4d(x, y, z, w);
    }

    @TRADEService
    @Action
    public boolean goToEEPose(String poseName) {
        return true;
    }
}
