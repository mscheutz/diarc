/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.pr2;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.navigation.NavigationAction;
import edu.tufts.hrilab.navigation.NavigationActionManager;
import edu.tufts.hrilab.diarcros.common.Amcl;
import edu.tufts.hrilab.diarcros.common.BaseControllerCommandPub;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovariance;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseStamped;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Twist;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;
import edu.tufts.hrilab.diarcros.msg.move_base_msgs.MoveBaseGoal;
import edu.tufts.hrilab.diarcros.pr2.Pr2MoveBaseNode;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import java.util.ArrayList;
import java.util.List;

public class PR2BaseComponent extends DiarcComponent implements PR2BaseInterface {

  // ROS connections
  private Pr2MoveBaseNode moveBaseNode;
  private BaseControllerCommandPub baseCmdPub;
  private Amcl amclNode;
  private volatile boolean poseNodeReady;
  // velocity params
  private final double defaultRotVel = 0.75;
  private final double defaultTransVel = 0.25;
  private double nominalRV = 0.0;
  private double nominalTV = 0.0;
  // laser info
  private boolean openFront = true;
  private boolean openLeft = true;
  private boolean openRight = true;
  private boolean safeFront = true;
  private boolean safeLeft = true;
  private boolean safeRight = true;
  private boolean setCritDist = false;
  private double criticalDist;
  private Pose initPose;                     //pose to seed localizer

  // motion action manager
  NavigationActionManager manager = new NavigationActionManager();

  private static int moveBaseGoalSeq = 0;

  /**
   * PR2BaseComponent constructor.
   */
  public PR2BaseComponent() {

    //ROS connections
    moveBaseNode = new Pr2MoveBaseNode();
    baseCmdPub = new BaseControllerCommandPub();
    amclNode = new Amcl();

    // wait for diarcros nodes to be ready
    moveBaseNode.waitForNode();
    baseCmdPub.waitForNode();
    amclNode.waitForNode();
  }

  @Override
  protected void init() {

    // if should set critical dist in laser component
    // TODO: how to do this properly in TRADE?
    if (setCritDist) {
      try {
        TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("setCritDist"));
        tsi.call(void.class, criticalDist);
      } catch (TRADEException e) {
        log.error("Could not setCritDist in laser component.", e);
      }
    }

    //seed initial pose if it's been set
    if (initPose != null) {
      PoseWithCovarianceStamped initPoseWithCovarianceStamped = new PoseWithCovarianceStamped();
      PoseWithCovariance pose_covar = new PoseWithCovariance();
      pose_covar.setPose(initPose);
      //pose_covar.covariance = new double[36];
      initPoseWithCovarianceStamped.setPose(pose_covar);

      amclNode.sendInitialpose(initPoseWithCovarianceStamped);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("criticalDist").hasArg().argName("dist").desc("critical obstacle distance (m)").build());
//    options.add(Option.builder("initialPose").numberOfArgs(2).argName("x y").desc("Initial seed pose for localizer").build());
    return options;
  }


  @Override
  protected void parseArgs(CommandLine cmdLine) {
//      if (cmdLine.hasOption("initialPose")) {
//        try {
//          initPose = new Pose();
//          initPose.setPosition(new Point(Double.parseDouble(args[++i]), Double.parseDouble(args[++i]), 0.0));
//          initPose.setOrientation(eulerToQuaternion(0.0, 0.0, Double.parseDouble(args[++i])));
//        } catch (Exception e) {
//          log.error("Invalid URI. Using default URI. " + e);
//        }
//        found = true;
//      }

    if (cmdLine.hasOption("criticalDist")) {
      try {
        setCritDist = true;
        criticalDist = Double.parseDouble(cmdLine.getOptionValue("criticalDist"));
      } catch (Exception e) {
        log.error("criticalDist not valid double value. " + e);
      }
    }
  }

  // ***********************************************************************
  // Methods available to remote objects via RMI
  // ***********************************************************************
  // ======== Navigation Interface ======================

  /**
   * Move to a global location.
   *
   * @param xdest  the x-coordinate of the destination
   * @param ydest  the y-coordinate of the destination
   * @param quat_x x quaternion value
   * @param quat_y y quaternion value
   * @param quat_z z quaternion value
   * @param quat_w w quaternion value
   * @return an identifying timestamp for the move action
   */
  @Override
  public long moveTo(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w) {
    PR2Action newAction = new PR2Action(PR2Action.ActionType.MOVETO, new double[]{xdest, ydest, quat_x, quat_y, quat_z, quat_w});
    manager.initiateAction(newAction);
    log.info("moveTo id: " + newAction.getID());
    return newAction.getID();
  }

  /**
   * Move to a global location.
   *
   * @param xdest the x-coordinate of the destination
   * @param ydest the y-coordinate of the destination
   * @return an identifying timestamp for the move action
   */
  @Override
  public long moveTo(double xdest, double ydest) {
    PR2Action newAction = new PR2Action(PR2Action.ActionType.MOVETOXY, new double[]{xdest, ydest});
    manager.initiateAction(newAction);
    log.info("moveTo id: " + newAction.getID());
    return newAction.getID();
  }

  /**
   * Move forward a specified distance.
   *
   * @param dist the distance (in meters) to move
   * @return an identifying timestamp for the move action
   */
  @Override
  public long moveDist(double dist) {
    PR2Action newAction = new PR2Action(PR2Action.ActionType.MOVEDIST, new double[]{dist});
    manager.initiateAction(newAction);
    return newAction.getID();
  }

  /**
   * Check status of current Motion. Soon this will be obsoleted, as the
   * motion commands will notify Action of their completions.
   *
   * @param aid the identifying timestamp of the action to check
   * @return the status found for the indicated action
   */
  @Override
  public ActionStatus checkMotion(long aid) {
    ActionStatus status = manager.getStatus(aid);
    return status;
  }

  /**
   * Cancel current Motion.
   *
   * @param aid the identifying timestamp of the action to cancel
   * @return true if action was canceled, false otherwise (i.e., if that
   * action ID was not active)
   */
  @Override
  public boolean cancelMotion(long aid) {
    return manager.cancel(aid);
  }

  // ============= Orientation Interface ==================

  /**
   * Turn to a global heading.
   *
   * @param tdest the global heading (in radians) to which to turn
   * @return an identifying timestamp for the turn action
   */
  @Override
  public long turnTo(double tdest) {
    PR2Action newAction = new PR2Action(PR2Action.ActionType.TURNTO, new double[]{tdest});
    manager.initiateAction(newAction);
    return newAction.getID();
  }

  /**
   * Turn a specified distance.
   *
   * @param dist the distance (in radians) to turn
   * @return an identifying timestamp for the turn action
   */
  @Override
  public long turnDist(double dist) {
    PR2Action newAction = new PR2Action(PR2Action.ActionType.TURNDIST, new double[]{dist});
    manager.initiateAction(newAction);
    return newAction.getID();
  }

  // ================ Velocity Interface =========================

  /**
   * Get translational velocity.
   *
   * @return the most recent TV reading (m/sec).
   */
  @Override
  public double getTV() {
    //TODO: get actual reading from ROS
    return nominalTV;
  }

  /**
   * Get rotational velocity.
   *
   * @return the most recent RV reading (rad/sec).
   */
  @Override
  public double getRV() {
    //TODO: get actual reading from ROS
    return nominalRV;
  }

  /**
   * Get translational and rotational velocity.
   *
   * @return the most recent velocity readings (m/sec and rad/sec).
   */
  @Override
  public double[] getVels() {
    //TODO: get actual reading from ROS
    return new double[]{nominalTV, nominalRV};
  }

  /**
   * Get the default velocities used by VelocityComponent functions.
   *
   * @return the default velocities (m/sec and rad/sec).
   */
  @Override
  public double[] getDefaultVels() {
    return new double[]{defaultTransVel, defaultRotVel};
  }

  /**
   * Stop.
   */
  @Override
  public void stop() {
    //
    nominalTV = 0.0;
    nominalRV = 0.0;
    PR2Action newAction = new PR2Action(PR2Action.ActionType.STOP);
    manager.initiateAction(newAction);
  }

  /**
   * Set translational velocity.
   *
   * @param tv the new TV (m/sec)
   * @return true if there's nothing in front of the robot, false
   * otherwise.
   */
  @Override
  public boolean setTV(double tv) {
    //
    return setVels(tv, nominalRV);
  }

  /**
   * Set rotational velocity.
   *
   * @param rv the new RV (rad/sec)
   * @return true if there's nothing on that side, false otherwise.
   */
  @Override
  public boolean setRV(double rv) {
    //
    return setVels(nominalTV, rv);
  }

  /**
   * Set both velocities.
   *
   * @param tv the new TV (m/sec)
   * @param rv the new RV (rad/sec)
   * @return true if there's nothing in front of the robot, false
   * otherwise.
   */
  @Override
  public boolean setVels(double tv, double rv) {
    nominalTV = tv;
    nominalRV = rv;

    //
    boolean isSafe = checkWithLRF(new double[]{tv, rv}, new double[]{0, 0});

    PR2Action newAction = new PR2Action(PR2Action.ActionType.SETVELS, new double[]{tv, rv});
    manager.initiateAction(newAction);
    return isSafe;
  }

  // ================ Position Methods ==========================

  @Override
  public double[] getPoseGlobalQuat() {
    Pose pose = amclNode.getAmclPose().getPose().getPose();
    return new double[]{pose.getPosition().getX(), pose.getPosition().getY(),
            pose.getOrientation().getX(), pose.getOrientation().getY(),
            pose.getOrientation().getZ(), pose.getOrientation().getW()};
  }

  // ================ Helper Methods/Classes =============================
  //TODO: move these classes to a generic/accesible location ??

  /**
   * Convert from quaternion orientation to euler. Currently only returns a
   * single theta (rad) in the x-y plane.
   * <p>
   * EAK: all 3 euler angles are calculated, so this method could be generalized.
   * TODO: use methods in RotationHelpers
   * NOTE: x and y might still be switched!
   *
   * @param orient - Quaternion
   * @return theta (rad)
   */
  private double quaternionToTheta(Quaternion orient) {
    double x, y, z;

    double sqw = orient.getW() * orient.getW();
    double sqx = orient.getX() * orient.getX();
    double sqy = orient.getY() * orient.getY();
    double sqz = orient.getZ() * orient.getZ();
    double unit = sqx + sqz + sqy + sqw; // if normalised is one, otherwise is correction factor
    double test = orient.getX() * orient.getZ() + orient.getY() * orient.getW();
    if (test > 0.499 * unit) { // singularity at north pole
      z = 2 * Math.atan2(orient.getX(), orient.getW());
      x = Math.PI / 2;
      y = 0;
    } else if (test < -0.499 * unit) { // singularity at south pole
      z = -2 * Math.atan2(orient.getX(), orient.getW());
      x = -Math.PI / 2;
      y = 0;
    } else {
      z = Math.atan2(2 * orient.getZ() * orient.getW() - 2 * orient.getX() * orient.getY(), sqx - sqz - sqy + sqw);
      x = Math.asin(2 * test / unit);
      y = Math.atan2(2 * orient.getX() * orient.getW() - 2 * orient.getZ() * orient.getY(), -sqx + sqz - sqy + sqw);
    }

    //System.out.println("x y z w " + orient.getX() + " " + orient.getY() + " " + orient.getZ() + " " + orient.getW());
    //System.out.println("q2e x: " + x + " y: " + y + " z: " + z);
    return z;
  }

  /**
   * Convert orientation on x-y plane to a quaternion orientation.
   *
   * @return Quaternion orientation
   */
  private Quaternion rotationToQuaternion(final double rot) {
    return eulerToQuaternion(0.0, 0.0, rot);
  }

  /**
   * Convert euler orientation (in radians) to quaternion orientation.
   * TODO: this should be replaced with methods in RotationHelpers
   * EAK: x and y might still be switched!
   *
   * @param x - rotation around x-axis
   * @param y - rotation around y-axis
   * @param z - rotation around z-axis
   * @return
   */
  private Quaternion eulerToQuaternion(final double x, final double y, final double z) {

    // Assuming the angles are in radians.
    double c1 = Math.cos(x / 2);
    double s1 = Math.sin(x / 2);
    double c2 = Math.cos(z / 2);
    double s2 = Math.sin(z / 2);
    double c3 = Math.cos(y / 2);
    double s3 = Math.sin(y / 2);
    double c1c2 = c1 * c2;
    double s1s2 = s1 * s2;

    Quaternion quat = new Quaternion();

    quat.setW(c1c2 * c3 - s1s2 * s3);
    quat.setX(c1c2 * s3 + s1s2 * c3);
    quat.setY(s1 * c2 * c3 + c1 * s2 * s3);
    quat.setZ(c1 * s2 * c3 - s1 * c2 * s3);

    //System.out.println("x y z w " + quat.x + " " + quat.y + " " + quat.z + " " + quat.getW());
    return quat;
  }

  /**
   * Check if the specified velocity values are safe, and modify them if they aren't.
   *
   * @param curr_vel - [trans(m/s) rot(rad/s)]
   * @param safe_vel - [trans(m/s) rot(rad/s)]
   * @return - true if current velocities are safe (ie, no adjustments made).
   */
  private boolean checkWithLRF(final double[] curr_vel, double[] safe_vel) {
    boolean[] safes = null;
    boolean[] opens = null;
    safe_vel[0] = curr_vel[0];
    safe_vel[1] = curr_vel[1];
    boolean safe = true;

    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getSafeSpaces"));
      safes = tsi.call(boolean[].class);

      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getOpenSpaces"));
      opens = tsi.call(boolean[].class);

    } catch (TRADEException e) {
      log.error("Could not get safe and open spaces from LRF.", e);
    }

    if (safes == null || opens == null) {
      log.error("Error checking LRF...setting velocites to 0");
      // set velocities to 0
      safe_vel[0] = 0.0;
      safe_vel[1] = 0.0;
      return false;
    }
    safeRight = safes[0];
    safeFront = safes[1];
    safeLeft = safes[2];
    openRight = opens[0];
    openFront = opens[1];
    openLeft = opens[2];
    if ((curr_vel[0] > 0) && (!safeFront)) {
      safe_vel[0] = 0.0;
      safe = false;
    }
    if (!safeLeft && !safeRight && !safeFront) {
      // have to allow it to turn, then
    } else if ((curr_vel[1] > 0) && (!safeLeft)) {
      safe_vel[1] = 0.0;
      safe = false;
    } else if ((curr_vel[1] < 0) && (!safeRight)) {
      safe_vel[1] = 0.0;
      safe = false;
    }

    return safe;
  }

  private class PR2Action extends NavigationAction {

    public PR2Action(ActionType t) {
      super(t);
    }

    public PR2Action(ActionType t, double[] args) {
      super(t, args);
    }

    public PR2Action(ActionType t, String[] args) {
      super(t, args);
    }

    public PR2Action(ActionType t, long[] args) {
      super(t, args);
    }

    // =====================================================
    // ============== Run methods ==========================
    public void run() {
      switch (type) {
        //case NONE:
        //case MOVETHROUGH:
        case MOVETO:
          moveTo(dargs[0], dargs[1], dargs[2], dargs[3], dargs[4], dargs[5]);
          break;
        case MOVETOXY:
          moveTo(dargs[0], dargs[1]);
          break;
        case MOVEDIST:
          moveDist(dargs[0]);
          break;
        //case TIMEMOVE:
        case MOVETOREL:
          moveToRel(dargs[0], dargs[1]);
          break;
        case TURNTO:
          turnTo(dargs[0]);
          break;
//                case TURNTOPOINT:
//                    turnToPoint(dargs[0], dargs[1]);
//                    break;
        case TURNDIST:
          turnDist(dargs[0]);
          break;
        //case TIMETURN:
        case STOP:
          stop();
          break;
        case SETVELS:
          setVelocity(dargs[0], dargs[1]);
          break;
        case FOLLOWWALL:
        case TRAVERSE:
        case APPROACHVISREF:
        case APPROACHVISCOLOR:
      }

    }

    private void sendMoveBaseGoal(String frame_id, double x_loc, double y_loc, double z_loc,
                                  double x_orient, double y_orient, double z_orient, double w_orient) {

      //increment unique goal seq
      ++moveBaseGoalSeq;

      //submit goal
      MoveBaseGoal goal = new MoveBaseGoal();
      PoseStamped target_pose = new PoseStamped();
      target_pose.getHeader().setStamp(amclNode.getCurrentTime());
      target_pose.getHeader().setSeq(moveBaseGoalSeq);
      target_pose.getHeader().setFrameId(frame_id);
      Pose pose = new Pose();
      Point position = new Point();
      position.setX(x_loc);
      position.setY(y_loc);
      position.setZ(z_loc);
      pose.setPosition(position);
      Quaternion orientation = new Quaternion();
      orientation.setX(x_orient);
      orientation.setY(y_orient);
      orientation.setZ(z_orient);
      orientation.setW(w_orient);
      pose.setOrientation(orientation);
      target_pose.setPose(pose);
      goal.setTargetPose(target_pose);

      try {
        moveBaseNode.sendMoveBaseMsgsMoveBaseGoal(goal);
      } catch (RosException e) {
        log.error("Error trying to sendMoveBaseGoal.", e);
      }
      setStatus(ActionStatus.PROGRESS);

      //wait for goal to finish
      try {
        moveBaseNode.waitForMoveBaseMsgsMoveBaseResult();
      } catch (InterruptedException e) {
        log.error("Interrupted while waiting for sendMoveBaseGoal to finish.", e);
      }

      //get result
      SimpleClientGoalState state = moveBaseNode.getMoveBaseMsgsMoveBaseState();
      if (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED) {
        setStatus(ActionStatus.SUCCESS);
      } else {
        setStatus(ActionStatus.FAIL);
      }
    }

    private void moveToRel(double xdest, double ydest) {
      // get current orientation
      Pose pose = amclNode.getAmclPose().getPose().getPose();
      //send back current orientation
      Quaternion orientation = pose.getOrientation();

      // submit goal
      sendMoveBaseGoal("base_link", xdest, ydest, 0, orientation.getX(),
              orientation.getY(), orientation.getZ(), orientation.getW());
    }

    private void moveTo(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w) {
      // submit goal
      sendMoveBaseGoal("map", xdest, ydest, 0, quat_x, quat_y, quat_z, quat_w);
    }

    private void moveTo(double xdest, double ydest) {
      // get current orientation
      Pose pose = amclNode.getAmclPose().getPose().getPose();
      //send back current orientation
      Quaternion orientation = pose.getOrientation();

      // submit goal
      sendMoveBaseGoal("map", xdest, ydest, 0, orientation.getX(),
              orientation.getY(), orientation.getZ(), orientation.getW());
    }

    private void moveDist(double dist) {
      // submit goal
      sendMoveBaseGoal("base_link", dist, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    }

    private void turnTo(double tdest) {
      // calculate new quaternion
      Quaternion orient = rotationToQuaternion(tdest);

      // get pose info
      Pose pose = amclNode.getAmclPose().getPose().getPose();

      //send back current global position
      Point pos = new Point();
      pos.setX(pose.getPosition().getX());
      pos.setY(pose.getPosition().getY());
      pos.setZ(pose.getPosition().getZ());

      // submit goal
      sendMoveBaseGoal("map", pos.getX(), pos.getY(), pos.getZ(),
              orient.getX(), orient.getY(), orient.getZ(), orient.getW());
    }

    private void turnDist(double dist) {
      // get orientation
      Quaternion quat = rotationToQuaternion(dist);

      // submit goal
      sendMoveBaseGoal("base_link", 0.0, 0.0, 0.0, quat.getX(), quat.getY(), quat.getZ(), quat.getW());
    }

    //        private void turnToPoint(double xdest, double ydest) {
//            //find angle to turn
//            //double theta = getHeadingFrom(0, 0, xdest, ydest);
//
//            //submit goal
//            Quat4d quat = rotationToQuaternion(dist);
//            moveBaseNode.submitGoal("base_link", 0.0, 0.0, 0.0, quat.x, quat.y, quat.z, quat.getW());
//            stat = ActionStatus.PROGRESS;
//
//            //wait for goal to finish
//            moveBaseNode.waitForGoal();
//
//            //get result
//            SimpleClientGoalState state = moveBaseNode.getState();
//            if (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED) {
//                stat = ActionStatus.SUCCESS;
//            } else {
//                stat = ActionStatus.FAIL;
//            }
//        }
    private void stop() {
      //submit goal
      Twist twist = new Twist();
      twist.setLinear(new Vector3(0, 0.0, 0.0));
      twist.setAngular(new Vector3(0.0, 0.0, 0));
      baseCmdPub.sendBaseControllerCommand(twist);

      //set status
      setStatus(ActionStatus.SUCCESS);
    }

    private void setVelocity(double tv, double rv) {
      //System.out.println("setvels: " + tv + " " + rv);
      double[] nominalVel = new double[]{tv, rv};
      double[] safeVel = new double[2];

      while (getRunFlag()) {
        //check LRF to get safe velocities
        checkWithLRF(nominalVel, safeVel);
        //System.out.println("setvels: " + tv + " " + rv + " " + safeVel[0] + " " + safeVel[1]);

        //submit goal
        Twist twist = new Twist();
        twist.setLinear(new Vector3(safeVel[0], 0.0, 0.0));
        twist.setAngular(new Vector3(0.0, 0.0, safeVel[1]));
        baseCmdPub.sendBaseControllerCommand(twist);

        //set status
        setStatus(ActionStatus.PROGRESS);

        try {
          Thread.sleep(10);
        } catch (Exception e) {
        }
      }

      //System.out.println("setvels exiting: " + tv + " " + rv);
      setStatus(ActionStatus.CANCEL);
    }

    // ======================================================
    // ================= Cancel Methods =====================
    public boolean cancel() {
      /*
       if (stat != ActionStatus.PROGRESS) {
       return false;
       }
       */

      setRunFlag(false);

      switch (type) {
        //case NONE:
        //case MOVETHROUGH:
        case MOVETO:
          cancelMoveBaseGoal();
          break;
        case MOVEDIST:
          cancelMoveBaseGoal();
          break;
        //case TIMEMOVE:
        case MOVETOREL:
          cancelMoveBaseGoal();
          break;
        case TURNTO:
          cancelMoveBaseGoal();
          break;
        case TURNTOPOINT:
          cancelMoveBaseGoal();
          break;
        case TURNDIST:
          cancelMoveBaseGoal();
          break;
        //case TIMETURN:
        case STOP:
          break;
        case SETVELS:
          break;
        //case FOLLOWWALL:
        //case TRAVERSE:
        //case APPROACHVISREF:
        //case APPROACHVISCOLOR:
      }

      setStatus(ActionStatus.CANCEL);
      return true;

    }

    private void cancelMoveBaseGoal() {
      //TODO: do I need to check to make sure this goal has been submitted before cancelling?
      try {
        moveBaseNode.cancelMoveBaseMsgsMoveBaseGoal();
      } catch (RosException e) {
        log.error("Exception trying to cancel move base goal.", e);
      }
    }
  }

}
