/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.moveit;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.List;

public class MockMoveItComponent extends DiarcComponent implements MoveItInterface {
  protected boolean shouldSimExecTime = false; // should the primitive actions simulate execution times
  protected long simExecTimeout = 1000;

  protected boolean grasping = false;

  public MockMoveItComponent() {
    super();
  }

  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("simExecTime").desc("Simulate execution time").build());
    options.addAll(super.additionalUsageInfo());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    super.parseArgs(cmdLine);
    if (cmdLine.hasOption("simExecTime")) {
      shouldSimExecTime = true;
    }
  }
  @Override
  public void enableCarryingConstraints() {
    log.info("[enableCarryingConstraints] method entered.");
  }

  @Override
  public void disableCarryingConstraints() {
    log.info("[disableCarryingConstraints] method entered.");
  }

  @Override
  public boolean moveTo(String group_name, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
    log.info("[moveTo] method entered.");
    simExecTime();
    return true;
  }

  @Override
  public boolean moveTo(String group_name, Point3d point, Quat4d orientation) {
    log.info("[moveTo] method entered.");
    simExecTime();
    return true;
  }

  @Override
  public Justification moveTo(String group_name, Symbol objectRef) {
    log.info("[moveTo] method entered.");
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveTo(String group_name, Symbol objectRef, List<? extends Term> grasp_constraints) {
    log.info("[moveTo] method entered. grasp_constraints: " + grasp_constraints);
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveToRelative(String group_name, Point3d point, Quat4d orientation) {
    log.info("[moveToRelative] method entered.");
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public Justification graspObject(String groupName, Symbol refId, float position) {
    log.info("[graspObject] method entered.");
    simExecTime();
    grasping = true;
    return new ConditionJustification(true);
  }

  @Override
  public Justification releaseObject(String groupName, Symbol refId, float position) {
    log.info("[releaseObject] method entered.");
    simExecTime();
    grasping = false;
    return new ConditionJustification(true);
  }

  @Override
  public Pair<Point3d, Quat4d> getEEPose(String group_name) {
    simExecTime();
    return null;
  }

  @Override
  public boolean moveToCartesian(String group_name, Point3d point, Quat4d orientation) {
    simExecTime();
    return true;
  }

  @Override
  public boolean moveToJointPositions(String[] jointNames, double[] positions) {
    return true;
  }

  @Override
  public boolean moveToJointPositions(String[] jointNames, double[] positions, double[] velocities, double[] efforts) {
    return true;
  }

  @Override
  public boolean moveToJointPosition(String jointName, double position) {
    log.info("[moveToJointPosition] " + jointName + " " + position);
    return true;
  }

  @Override
  public Pair<Point3d, Quat4d> getPose(String link_name) {
    log.info("[pointHeadTo] location method entered.");
    simExecTime();
    return new MutablePair<>(new Point3d(1, 1, 1), new Quat4d(0, 0, 0, 1));
  }

  @Override
  public boolean pointTo(String group_name, Symbol objectRef) {
    simExecTime();
    log.info("[pointTo] object method entered.");
    return true;
  }

  @Override
  public boolean pointTo(String group_name, Point3d targetLocation) {
    simExecTime();
    log.info("[pointTo] location method entered.");
    return true;
  }

  @Override
  public boolean recordPose(Symbol pose_name) {
    log.info("[recordPose] " + pose_name);
    simExecTime();
    return true;
  }

  @Override
  public boolean saveEEPosesToFile(String filename) {
    log.info("[saveEEPosesToFile] " + filename);
    simExecTime();
    return true;  }

  @Override
  public void loadEEPosesFromFile(String filename) {
    log.info("[loadEEPosesFromFile] " + filename);
    simExecTime();
  }

  @TRADEService
  @Action
  public boolean recordEEPose(Symbol poseName) {
    log.info("[recordEE] " + poseName);
    simExecTime();
    return true;
  }

  @TRADEService
  @Action
  public boolean goToEEPose(Symbol poseName) {
    log.info("[goToEEPose] " + poseName);
    simExecTime();
    return true;
  }

  @Override
  public boolean goToPose(Symbol pose_name) {
    log.info("[goToPose] " + pose_name);
    simExecTime();
    return true;
  }

  @Override
  public boolean goToStartPose() {
    simExecTime();
    return true;
  }

  @Override
  public boolean goToStartPose(boolean safe) {
    log.info("[goToStartPose]");
    return true;
  }

  @Override
  public boolean savePosesToFile(String filename) {
    log.info("[savePosesToFile] " + filename);
    return true;
  }

  //TODO:brad: maybe implement this?
  @Override
  public void loadPosesFromFile(String filename) {
    log.info("[loadPosesFromFile] " + filename);
  }

  @Override
  public boolean goToPose(String groupName, Symbol poseName) {
    log.info("[goToPose] " + poseName + " " + groupName);
    simExecTime();
    return true;
  }

  @Override
  public boolean publishPointCloudToRos(byte[] depthData) {
    log.info("[publishPointCloudToRos]");
    return true;
  }

  @Override
  public void startRecordingTrajectory(String trajectory_name) {
    log.info("[startRecordingTrajectory] " + trajectory_name);
    simExecTime();
  }

  @Override
  public void stopRecordingTrajectory() {
    log.info("[stopRecordingTrajectory]");
    simExecTime();
  }

  @Override
  public boolean executeTrajectory(String trajectory_name) {
    log.info("[publishPointCloudToRos] " + trajectory_name);
    simExecTime();
    return true;
  }

  @Override
  public boolean saveTrajectoriesToFile(String filename) {
    log.info("[saveTrajectoriesToFile] " + filename);
    return true;
  }

  @Override
  public void loadTrajectoriesFromFile(String filename) {
    log.info("[loadTrajectoriesFromFile] " + filename);
  }

  @Override
  public Justification closeGripper(String groupName) {
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public Justification openGripper(String groupName) {
    simExecTime();
    grasping = false;
    return new ConditionJustification(true);
  }

  @Override
  public boolean moveGripper(String groupName, float position) {
    simExecTime();
    return true;
  }

  @Override
  public float getGripperPosition(String groupName) {
    return 0;
  }

  @Override
  public Justification learn(Term learningTerm) {
    return null;
  }

  @Override
  public Justification unlearn(Term learningTerm) {
    return null;
  }

  @Override
  public Justification pressObject(String group_name, Point3d object_location, Quat4d object_orientation){
    log.info("[pressObject]");
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public Justification pressObject(String group_name, Symbol refID){
    log.info("[pressObject]");
    simExecTime();
    return new ConditionJustification(true);
  }

  private void simExecTime() {
    if (shouldSimExecTime) {
      try {
        Thread.sleep(simExecTimeout);
      } catch (InterruptedException e) {
      }
    }
  }
}
