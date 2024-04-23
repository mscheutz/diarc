/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.pr2;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.moveit.MoveItInterface;

import javax.vecmath.Point3d;

public interface PR2Interface extends MoveItInterface {

  /**
   * Move the torso to desired joint position.
   * TODO: replace this with the more general setJointPosition(String joint, double position) method in MoveItComponent
   *
   * @param position all the way down is 0.0, and all the way up is approximately 0.3
   * @return
   */
  @TRADEService
  @Action
  boolean setTorsoPosition(double position);

  /**
   * Point head in direction of target object in base_link coordinate frame.
   * Alternative method to pointHeadTo that takes in POWER ref instead of MemoryObject.
   *
   * @param objectRef
   * @return
   */
  @TRADEService
  @Action
  boolean pointHeadTo(Symbol objectRef);

  /**
   * Point head in direction of target point in base_link coordinate frame.
   *
   * @param target_point
   * @return
   */
  @TRADEService
  @Action
  boolean pointHeadTo(Point3d target_point);

  @TRADEService
  boolean goToPoseNoPlanning(String pose_name);

}
