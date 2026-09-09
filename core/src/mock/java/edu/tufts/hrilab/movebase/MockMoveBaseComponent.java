/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.movebase;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.interfaces.MoveBaseInterface;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.*;

public class MockMoveBaseComponent extends DiarcComponent implements MoveBaseInterface {
  boolean shouldSimExecTime = false; // should the primitive actions simulate execution times
  long simExecTimeout = 3000; // timeout to simulate execution time

  //used to manage the previous and current location, since checkAt doesn't have outside information to handle this. important since checkAt observes facts used in preconditions and effects of asl actions as well
  private Symbol currentLocation = Factory.createSymbol("location_0", "location");

  private Pair<Point3d, Quat4d> currRobotPose;

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("simExecTime").desc("Simulate execution time").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("simExecTime")) {
      shouldSimExecTime = true;
    }
  }

  @Override
  protected void init() {
  }

  @Override
  public void shutdownComponent() {
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  ///////////////////////////////////////////////////////////////////////////////

  @Override
  public void setPose(Point3d location, Quat4d orientation) {
    simExecTime();
  }

  @Override
  public Pair<Point3d, Quat4d> getPose() {
    simExecTime();
//    return new double[0];
    // Mock data for demonstration purposes
    Point3d meterPosition;
    Quat4d orientationResult;

    if (currRobotPose != null) {
      // If currRobotPose is not null, use its position and orientation
      meterPosition = currRobotPose.getLeft();
      orientationResult = currRobotPose.getRight();

      log.info("orientationResult from getPoseGlobalQuat is {}", orientationResult);
    } else {
      // If currRobotPose is null, fallback to another method to get the pose
      PoseReference positionResult = getPoseReference(currentLocation);
      meterPosition = positionResult.getPosition();
      orientationResult = positionResult.getOrientation();
    }

    return new MutablePair<>(meterPosition, orientationResult);
  }

  @Override
  public Justification goToLocation(Symbol location, boolean wait) {
    simExecTime();
//    if (getLocationPose(location) != null) {
//      currentLocation = location;
//      return new ConditionJustification(true);
//    }
//    return new ConditionJustification(false);
    currentLocation = location;
    return new ConditionJustification(true);
  }

//  @Override
  public Justification goToLocation(Symbol location) {
    return goToLocation(location, true);
  }

  @Override
  public Justification goToLocation(Symbol desiredLocation, Symbol initialLocation) {
    simExecTime();
    currentLocation = desiredLocation;
    return null;
  }

  @Override
  public Justification goToLocation(Point3d location, Quat4d orientation, boolean wait) {
    simExecTime();
    currRobotPose = new MutablePair<>(location, orientation);
    return new ConditionJustification(true);
  }

  @Override
  public Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait) {
    simExecTime();
    currRobotPose = new MutablePair<>(new Point3d(xdest, ydest, 0.0), new Quat4d(quat_x, quat_y, quat_z, quat_w));
    return new ConditionJustification(true);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation) {
    simExecTime();
    currentLocation = desiredLocation;
    return new ConditionJustification(true);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation, Symbol initialLocation) {
    simExecTime();
    currentLocation = desiredLocation;
    return new ConditionJustification(true);
  }

  @Override
  public Justification stop() {
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public Justification isMoving() {
    simExecTime();
    return new ConditionJustification(true);
  }

  @Override
  public List<Map<Variable, Symbol>> checkAt(Term locationTerm) {
    simExecTime();
    List<Map<Variable, Symbol>> returnVal = new ArrayList<>();
    //todo: null check here is for the case where the location hasn't been initialized. required for managing the previous location.
    if (currentLocation == null || locationTerm.get(1).equals(currentLocation)) {
      returnVal.add(new HashMap<>());
    }
    return returnVal;
  }

  private void simExecTime() {
    if (shouldSimExecTime) {
      try {
        Thread.sleep(simExecTimeout);
      } catch (InterruptedException e) {
      }
    }
  }

  //todo: This assumes that the location consultant binds to a matrix4d, which is not always the case (e.g. spot)
  protected Matrix4d getLocationPose(Symbol refID) {
    Matrix4d locPose = null;
    try {
      locPose = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class,Class.class))
              .call(Matrix4d.class, refID, Matrix4d.class);
    } catch (TRADEException e) {
      log.error("[getLocationPose] exception getting Matrix4d from reference, returning null", e);
    }

    return locPose;
  }

  protected PoseReference getPoseReference(Symbol refID) {
    PoseReference ref = null;
    try {
      ref = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class,Class.class))
              .call(PoseReference.class, refID, PoseReference.class);
    } catch (TRADEException e) {
      log.error("[getLocationPose] exception getting PoseReference from reference, returning null", e);
    }

    return ref;
  }

}
