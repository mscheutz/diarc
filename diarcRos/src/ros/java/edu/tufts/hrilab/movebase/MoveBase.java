/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.movebase;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest;
import edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse;
import edu.tufts.hrilab.diarcros.msg.Time;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;
import edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import java.lang.*;

import edu.tufts.hrilab.diarcros.common.MoveBaseNode;
import edu.tufts.hrilab.diarcros.common.RosConfiguration;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.tufts.hrilab.fol.Factory;

import edu.tufts.hrilab.util.Util;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;

public class MoveBase {
  private MoveBaseNode base;

  private String mapFrame, baseFrame, myGroups;

  private final Lock moveLock = new ReentrantLock();

  protected static Logger log = LoggerFactory.getLogger(MoveBase.class);

  /**
   * Constructor.
   *
   * @param rc
   * @param mapFrame
   * @param baseFrame
   */
  public MoveBase(RosConfiguration rc, String mapFrame, String baseFrame, List<String> myGroups) {
    base = new MoveBaseNode(rc);
    base.waitForNode();
    this.mapFrame = mapFrame;
    this.baseFrame = baseFrame;
    this.myGroups = String.join(" ", myGroups);
  }

  /**
   * Get robot's current pose.
   *
   * @return
   */
  public Pose getPose() {
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTransform").argTypes(String.class, String.class).inGroups(myGroups));
      Matrix4d matTf = tsi.call(Matrix4d.class, mapFrame, baseFrame);
      Vector3d vec = new Vector3d();
      Quat4d quat = new Quat4d();
      matTf.get(vec);
      matTf.get(quat);

      // return pose value
      return new Pose(new Point(vec.x, vec.y, vec.z), new Quaternion(quat.x, quat.y, quat.z, quat.w));
    } catch (TRADEException e) {
      log.error("Error calling getTransform.", e);
      return null;
    }
  }

  /**
   * Go to a pose in the world.
   *
   * @param pose a goal pose
   */
  public Justification goToLocation(final Pose pose, boolean wait) {
    if (getPose() == null) {
      log.warn("Robot pose has not been initialized. Navigation will likely fail");
    }
    PoseStamped stamped = new PoseStamped();
    stamped.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
    stamped.getHeader().setFrameId(mapFrame);
    stamped.setPose(pose);
    MoveBaseGoal goal = new MoveBaseGoal();
    goal.setTargetPose(stamped);

    log.debug("[goTo] waiting on lock...");
    moveLock.lock();
    log.debug("[goTo] acquired lock.");
    try {
      // cancel any currently executing goals before submitting a new one
      stop();
      log.debug("clearing costmap");
      base.callClearCostmaps(new EmptyRequest(), new EmptyResponse());
      log.debug("cleared costmap");
      log.debug("sending movebase");
      base.sendMoveBaseGoal(goal);
    } catch (RosException e) {
      log.error("[goTo] failed goal: " + base.getMoveBaseGoal().getGoalId().getId(), e);
      return new ConditionJustification(false);
    } finally {
      moveLock.unlock();
    }

    if (wait) {
      log.debug("[goTo] waiting for goal(s) to finish...");
      try {
        base.waitForMoveBaseResult();
      } catch (InterruptedException e) {
        log.error("[goTo] interrupted while waiting for goal to finish: " + base.getMoveBaseGoal().getGoalId().getId(), e);
      }

      log.debug("[goTo] done waiting for goal(s) to finish: " + base.getMoveBaseGoal().getGoalId().getId());
      SimpleClientGoalState goalState = base.getMoveBaseState();
      if (goalState.getState() != SimpleClientGoalState.StateEnum.SUCCEEDED) {
        log.warn("[goTo] goal " + base.getMoveBaseGoal().getGoalId().getId() + " returned with state: " + goalState.getState());
        return new ConditionJustification(false, Factory.createNegatedPredicate(Factory.createPredicate("goal", goalState.getState().toString().toLowerCase())));
      } else {
        log.debug("[goTo] goal " + base.getMoveBaseGoal().getGoalId().getId() + " returned with state: " + goalState.getState());
      }
    }

    return new ConditionJustification(true);
  }

  /**
   * Stop the robot, and cancel any move base goals.
   *
   * @return
   */
  public Justification stop() {
    //log.debug("[stopRobot] waiting on lock...");
    moveLock.lock();
    //log.debug("[stopRobot] acquired lock.");
    try {
      if (base.getMoveBaseGoal() == null) {
        //log.debug("Current move base goal is null. Not cancelling any goals.");
        return new ConditionJustification(true);
      }

      if (!base.getMoveBaseState().isDone()) {
        //log.debug("[stopRobot] Cancelling goal: " + base.getMoveBaseGoal().getGoalId().getId() + " with status: " + base.getMoveBaseState());
        base.cancelAllMoveBaseGoals();

        // wait for cancelled goal to finish cancelling
        // NOTE: do not use waitForMoveBaseResult here because it will steal the wait for notification from the goTo method
        // and it will never be notified, causing a hanging goTo method that will never return
        //log.debug("[stopRobot] Waiting for goal to be done: " + base.getMoveBaseGoal().getGoalId().getId());
        while (!base.getMoveBaseState().isDone()) {
          Util.Sleep(10);
        }
        //log.debug("[stopRobot] Goal done: " + base.getMoveBaseGoal().getGoalId().getId() + " with status: " + base.getMoveBaseState());
      }
    } finally {
      moveLock.unlock();
    }

    return new ConditionJustification(true);
  }

  /**
   * Is robot currently moving.
   *
   * @return
   */
  public Justification isMoving() {
    moveLock.lock();
    boolean movingHolds = false;
    try {
      movingHolds = (base != null && !base.getMoveBaseState().isDone());
    } catch (Exception e) {
      log.error("Error checking isMoving.", e);
    } finally {
      moveLock.unlock();
    }
    return new ConditionJustification(movingHolds);
  }
}