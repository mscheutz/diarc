/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;


import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.moveit.MoveItInterface;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import javax.vecmath.Point3d;
import java.util.List;
import java.util.Map;


public interface FetchInterface extends MoveItInterface {

  /**
   * Move the torso to desired joint position.
   * @param position all the way down is 0.0, and all the way up is approximately 0.3
   * @return

   * 
   * TODO: replace this with more general setJointPosition(String joint, double position) method
   */
  @TRADEService
  @Action
  boolean setTorsoPosition(double position) ;

  /**
  * Point head in direction of target object in base_link coordinate frame.
  * @param target_object
  * @return

  */
  @TRADEService
  @Action
  boolean pointHeadTo(MemoryObject target_object) ;

  /**
  * Point head in direction of target object in base_link coordinate frame.
  * Alternative method to pointHeadTo that takes in POWER ref instead of MemoryObject.
  * @param objectRef
  * @return

  */
  @TRADEService
  @Action
  boolean pointHeadTo(Symbol objectRef) ;

  /**
  * Point head in direction of target point in base_link coordinate frame.
  *
  * @param target_point
  * @return

  */
  @TRADEService
  @Action
  boolean pointHeadTo(Point3d target_point) ;

  @TRADEService
  @Observes({"holding(?actor,?objectRef,?arm)", "grasping(?actor,?objectRef,?arm)"})
  List<Map<Variable, Symbol>> checkGrasping(Term graspingTerm);

  @TRADEService
  @Action
  Justification look(String direction);

  @TRADEService
  @Action
  Justification lookAround();

  /**
   * Gets the charge level of the fetch
   * @return The charge level of the fest (between 0-1) or -1 if battery state is null (likely not being published)
   */
  @TRADEService
  @Action
  float getChargeLevel();

  /**
   * Gets the specified breaker state.
   *
   * @param breakerName The name of the breaker. Currently, these are the relevant breakers: computer_breaker,
   *                    battery_breaker, supply_breaker, base_breaker, arm_breaker, gripper_breaker.
   * @return An enumerated FetchBreakerStates object representing the states of the breaker or null if robot state is
   * null/not being published or the breaker name cannot be found)
   */
  @TRADEService
  @Action
  FetchBreakerStates getBreakerState(String breakerName);

  /**
   * Sends a breaker command request to set the given breaker to the desired state.
   *
   * @param breakerName The name of the breaker. Current settable breakers: base_breaker, arm_breaker, gripper_breaker
   * @param state The desired state of the specified breaker. TRUE means ON and FALSE means OFF.
   * @return An enumerated FetchBreakerStates object representing the actual resultant state of the breaker (i.e. if the E-stop is on, and you try to turn on the breaker it will still be off).
   * A null return value likely means the breaker name is not valid/cannot be found.
   */
  @TRADEService
  @Action
  FetchBreakerStates setBreakerState(String breakerName, boolean state);
}
