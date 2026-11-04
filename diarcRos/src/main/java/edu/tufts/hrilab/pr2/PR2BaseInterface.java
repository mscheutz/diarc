/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.pr2;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.interfaces.NavigationInterface;
import edu.tufts.hrilab.interfaces.OrientationInterface;
import edu.tufts.hrilab.interfaces.VelocityInterface;

import ai.thinkingrobots.trade.TRADEService;

public interface PR2BaseInterface extends NavigationInterface, OrientationInterface, VelocityInterface {

  /**
   * Move to a global location.
   *
   * TODO: move this method to NavigationComponent interface.
   *
   * @param xdest the x-coordinate of the destination
   * @param ydest the y-coordinate of the destination
   * @param quat_x x quaternion value
   * @param quat_y y quaternion value
   * @param quat_z z quaternion value
   * @param quat_w w quaternion value
   * @return an identifying timestamp for the move action
   */
  @TRADEService
  @Action
  long moveTo(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w);

  /**
   * Alternative version of getPoseGlobal that returns x-y location as well as
   * quaternion orientation (x, y, quat_x, quat_y, quat_z, quat_w).
   * @return
   */
  @TRADEService
  @Action
  double[] getPoseGlobalQuat();

}
