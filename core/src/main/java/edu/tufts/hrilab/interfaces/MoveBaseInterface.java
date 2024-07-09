/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Condition;
import edu.tufts.hrilab.action.annotations.Effect;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;
import java.util.Map;

/**
 * Provides simplified access to the ROS move base component on the PR2.
 *
 * This interface is convenient when you want to be able to send
 * navigation goals without tracking their progress.
 *
 * @author Jeremiah Via jeremiah.via@gmail.com
 */
public interface MoveBaseInterface {

  @TRADEService
  @Action
  double[] getPoseGlobalQuat();

  @TRADEService
  @Action
  void setPoseGlobal(double x, double y, double theta);

  /**
   * Attempt to reach a named goal location.
   *
   * @param location - name of the goal location
   */
  @TRADEService
  @Action
  /*@Effect(
    effect={"at(?actor,?location)"},
    type = EffectType.SUCCESS,
    observable = {"at(?actor,?location"}
  )*/
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification goToLocation(Symbol location, boolean wait) ;

  /**
   * go to the location and then move forward until certain distance from object in front
   * @param location name of the desired location to navigate to
   * @return justification specifying if the robot navigated to the location
   */
  @TRADEService
  @Action
      /*@Effect(
      effect={"at(?actor,?location)"},
      type = EffectType.SUCCESS,
      observable = {"at(?actor,?location"}
  )*/
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification goToLocation(Symbol location) ;

  /**
   * go to the location and then move forward until certain distance from object in front
   * @param desiredLocation name of the desired location to navigate to
   * @param initialLocation name of the initial location to navigate from -- used for conditions and effects
   * @return justification specifying if the robot navigated to the location
   */
  @TRADEService
  @Action
  @Condition(
      condition={"at(?actor,?initialLocation)"},
      type= ConditionType.PRE,
      observable={"at(?actor,?initialLocation)"}
  )
  @Effect(
      effect={"at(?actor,?desiredLocation)", "not(at(?actor,?initialLocation))"},
      type = EffectType.SUCCESS,
      observable = {"at(?actor,?desiredLocation)", "not(at(?actor,?initialLocation))"}
  )
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification goToLocation(Symbol desiredLocation, Symbol initialLocation);

  /**
   * go to the point and orientation
   * @param xdest x point
   * @param ydest y point
   * @param quat_x x quaternion value
   * @param quat_y y quaternion value
   * @param quat_z z quaternion value
   * @param quat_w w quaternion value
   * @param wait should robot wait until reached desired point and orientation
   * @return justification if the robot reach the desired point and orientation
   */
  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait) ;

  /**
   * go to the location and then move forward until certain distance from object in front
   * @param location name of the desired location to navigate to
   * @return justification specifying if the robot navigated to the location
   */
  @TRADEService
  @Action
  @Effect(
    effect={"at(?actor,?location)"},
    type = EffectType.SUCCESS,
    observable = {"at(?actor,?location"}
  )
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification approachLocation(Symbol location) ;

  /**
   * go to the location and then move forward until certain distance from object in front
   * @param desiredLocation name of the desired location to navigate to
   * @param initialLocation name of the initial location to navigate from -- used for conditions and effects
   * @return justification specifying if the robot navigated to the location
   */
  @TRADEService
  @Action
  @Condition(
    condition={"at(?actor,?initialLocation)"},
    type= ConditionType.PRE,
    observable={"at(?actor,?initialLocation)"}
  )
  @Effect(
    effect={"at(?actor,?desiredLocation)", "not(at(?actor,?initialLocation))"},
    type = EffectType.SUCCESS,
    observable = {"at(?actor,?desiredLocation)", "not(at(?actor,?initialLocation))"}
  )
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification approachLocation(Symbol desiredLocation, Symbol initialLocation);

  @TRADEService
  @Action
  Justification stop() ;

  @TRADEService
  @Action
  Justification isMoving() ;

  /**
   * Observer to check if the robot is at the specified named location
   * @param locationTerm predicate to observe in form: at(?actor, ?location)
   * @return observation result
   */
  @TRADEService
  @Observes({"at(?actor,?location)"})
  List<Map<Variable, Symbol>> checkAt(Term locationTerm);
}
