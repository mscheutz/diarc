/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.pr2;

import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.lang3.tuple.Pair;

import java.util.List;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

/**
 *
 * @author Evan Krause
 */
public class MockPR2Component extends DiarcComponent implements PR2Interface {

  public MockPR2Component()  {
    super();
  }

  @Override
  public boolean moveTo(String group_name, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r)  {
    log.info("[moveTo] method entered.");
    return true;
  }

  @Override
  public boolean moveTo(String group_name, Point3d point, Quat4d orientation)  {
    log.info("[moveTo] method entered.");
    return true;
  }

  @Override
  public Justification moveTo(String group_name, Symbol objectRef)  {
    log.info("[moveTo] method entered.");
    return new ConditionJustification(true);
  }

  @Override
  public boolean moveToJointPositions(String[] jointNames, double[] positions) {
    return true;
  }

  @Override
  public boolean moveToJointPositions(String[] joint_names, double[] positions, double[] velocities, double[] efforts) {
    return true;
  }

  @Override
  public Justification moveTo(String group_name, Symbol objectRef, List<? extends Term> grasp_constraints)  {
    log.info("[moveTo] method entered. grasp_constraints: " + grasp_constraints);
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveToRelative(String group_name, Point3d point, Quat4d orientation)  {
    log.info("[moveToRelative] method entered.");
    return new ConditionJustification(true);
  }

  @Override
  public Justification graspObject(String groupName, Symbol refId, float position) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification releaseObject(String groupName, Symbol refId, float position) {
    return new ConditionJustification(true);
  }

  @Override
  public Pair<Point3d, Quat4d> getEEPose(String group_name) {
    return null;
  }

  @Override
  public Pair<Point3d, Quat4d> getPose(String link_name) {
    return null;
  }

  @Override
  public boolean moveToCartesian(String group_name, Point3d point, Quat4d orientation) {
    return true;
  }

  @Override
  public boolean pointHeadTo(Symbol objectRef)  {
    log.info("[pointHeadTo] object method entered.");
    return true;
  }

  @Override
  public boolean pointHeadTo(Point3d target_point)  {
    log.info("[pointHeadTo] location method entered.");
    return true;
  }

  @Override
  public boolean pointTo(String group_name, Symbol objectRef)  {
    log.info("[pointTo] object method entered.");
    return true;
  }
  
  @Override
  public boolean pointTo(String group_name, Point3d targetLocation)  {
    log.info("[pointTo] location method entered.");
    return true;
  }

  @Override
  public boolean setTorsoPosition(double position)  {
    log.info("[setTorsoPosition] method entered.");
    return true;
  }

  @Override
  public boolean goToStartPose(boolean safe)  {
    log.info("[goToStartPose] method entered.");
    return true;
  }

  @Override
  public boolean recordPose(Symbol pose_name)  {
    log.info("[recordPose]");
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
  public boolean goToPose(Symbol pose_name)  {
    log.info("[goToPose]");
    return true;
  }

  @Override
  public boolean goToStartPose() {
    return true;
  }

  @Override
    public boolean goToPose(String pose_name, Symbol group_name)  {
    log.info("[goToPose] group_name: " + group_name);
    return true;
  }

  @Override
  public boolean goToPoseNoPlanning(String pose_name)  {
    log.info("[goToPoseNoPlanning]");
    return true;
  }

  @Override
  public void startRecordingTrajectory(String trajectory_name)  {
    log.info("[startRecordingTrajectory]");
  }

  @Override
  public void stopRecordingTrajectory()  {
    log.info("[stopRecordingTrajectory]");
  }

  public boolean executeTrajectory(String trajectory_name)  {
    log.info("[executeTrajectory]");
    return true;
  }

  public boolean savePosesToFile(String filename)  {
    log.info("[savePosesToFile]");
    return true;
  }

  public void loadPosesFromFile(String filename)  {
    log.info("[loadPosesFromFile]");
  }

  public boolean saveTrajectoriesToFile(String filename)  {
    log.info("[saveTrajectoriesToFile]");
    return true;
  }

  public void loadTrajectoriesFromFile(String filename)  {
    log.info("[loadTrajectoriesFromFile]");
  }

  @Override
  public Justification closeGripper(String groupName) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification openGripper(String groupName) {
    return new ConditionJustification(true);
  }

  @Override
  public boolean moveGripper(String groupName, float position) {
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
  public boolean moveToJointPosition(String joint, double position) {
    return true;
  }

  @Override
  public void enableCarryingConstraints() {

  }

  @Override
  public void disableCarryingConstraints() {

  }

  @Override
  public boolean publishPointCloudToRos(byte[] data)  {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Justification learn(Term learningTerm) {
    return null;
  }

  @Override
  public Justification unlearn(Term learningTerm) {
    return null;
  }
}
