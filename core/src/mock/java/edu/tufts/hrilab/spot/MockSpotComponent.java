package edu.tufts.hrilab.spot;

import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.List;
import java.util.Map;

public class MockSpotComponent extends DiarcComponent implements SpotInterface {
  MockMoveBaseComponent moveBase = createInstance(MockMoveBaseComponent.class, "", false);

  @Override
  public Justification claim() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification release() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification powerOn() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification powerOff() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification stand() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification sit() {
    return new ConditionJustification(true);
  }

  @Override
  public boolean isLocalized() {
    return true;
  }

  @Override
  public Justification localizeWithReference(Symbol refId) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification localizeWithFiducial() {
    return new ConditionJustification(true);
  }

  @Override
  public Justification grabObject(Symbol refId) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveTo(String groupName, Point3d point, Quat4d orientation) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveTo(String groupName, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveTo(String groupName, Symbol refId) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveToRelative(String groupName, Point3d point, Quat4d orientation) {
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
  public Justification pointTo(String groupName, Symbol objectRef) {
    return new ConditionJustification(true);
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
    return new ConditionJustification(true);
  }

  @Override
  public Justification saveEEPosesToFile(String filename) {
    return new ConditionJustification(true);
  }

  @Override
  public void loadEEPosesFromFile(String filename) {

  }

  @Override
  public Justification savePosesToFile(String filename) {
    return new ConditionJustification(true);
  }

  @Override
  public void loadPosesFromFile(String filename) {

  }

  @Override
  public Justification goToPose(String groupName, Symbol poseName) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification goToPose(Symbol poseName) {
    return new ConditionJustification(true);
  }

  @Override
  public void startRecordingTrajectory(String trajectoryName) {

  }

  @Override
  public void stopRecordingTrajectory() {

  }

  @Override
  public Justification executeTrajectory(String trajectoryName) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification saveTrajectoriesToFile(String filename) {
    return new ConditionJustification(true);
  }

  @Override
  public void loadTrajectoriesFromFile(String filename) {

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
  public Justification moveGripper(String groupName, float meters) {
    return new ConditionJustification(true);
  }

  @Override
  public float getGripperPosition(String groupName) {
    return 0;
  }

  @Override
  public Justification pressObject(String group_name, Point3d object_location, Quat4d object_orientation) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification pressObject(String group_name, Symbol refID) {
    return new ConditionJustification(true);
  }

  @Override
  public Matrix4d getTransform(String srcFrame, String dstFrame) {
    Matrix4d mat = new Matrix4d();
    mat.setIdentity();
    return mat;
  }

  @Override
  public List<String> getCoordinateFrames() {
    return List.of();
  }

  @Override
  public void addLocalStaticTransform(String parent, String child, Matrix4d transform) {

  }

  @Override
  public Justification dock(Symbol dockId) {
    return new ConditionJustification(true);
  }

  @Override
  public Justification undock() {
    return new ConditionJustification(true);
  }

  @Override
  public void startMapping() {

  }

  @Override
  public void createWaypoint(Symbol name) {

  }

  @Override
  public void finishMapping() {

  }

  @Override
  public double[] getPoseGlobalQuat() {
    return moveBase.getPoseGlobalQuat();
  }

  @Override
  public void setPoseGlobal(double x, double y, double theta) {
    moveBase.setPoseGlobal(x, y, theta);
  }

  @Override
  public Justification goToLocation(Symbol location, boolean wait) {
    return moveBase.goToLocation(location, wait);
  }

  @Override
  public Justification goToLocation(Symbol location) {
    return moveBase.goToLocation(location);
  }

  @Override
  public Justification goToLocation(Symbol desiredLocation, Symbol initialLocation) {
    return moveBase.goToLocation(desiredLocation, initialLocation);
  }

  @Override
  public Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait) {
    return moveBase.goToLocation(xdest, ydest, quat_x, quat_y, quat_z, quat_w, wait);
  }

  @Override
  public Justification approachLocation(Symbol location) {
    return moveBase.approachLocation(location);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation, Symbol initialLocation) {
    return moveBase.approachLocation(desiredLocation, initialLocation);
  }

  @Override
  public Justification stop() {
    return moveBase.stop();
  }

  @Override
  public Justification isMoving() {
    return moveBase.isMoving();
  }

  @Override
  public List<Map<Variable, Symbol>> checkAt(Term locationTerm) {
    return moveBase.checkAt(locationTerm);
  }

  @Override
  public double getTV() {
    return 0;
  }

  @Override
  public double getRV() {
    return 0;
  }

  @Override
  public double[] getVels() {
    return new double[0];
  }

  @Override
  public double[] getDefaultVels() {
    return new double[0];
  }

  @Override
  public boolean setTV(double tv) {
    return true;
  }

  @Override
  public boolean setRV(double rv) {
    return true;
  }

  @Override
  public boolean setVels(double tv, double rv) {
    return true;
  }

  @Override
  public boolean setVels(double vel_x, double vel_y, double vel_rot) {
    return true;
  }
}
