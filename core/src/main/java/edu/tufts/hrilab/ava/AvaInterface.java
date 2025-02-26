package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Effect;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.ava.message.response.Obstacle;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.LidarInterface;
import edu.tufts.hrilab.interfaces.MoveBaseInterface;

import java.util.List;

public interface AvaInterface extends MoveBaseInterface, LidarInterface {

  @TRADEService
  void displayRobot(TRADEServiceInfo displayService);

  @Action
  @TRADEService
  Justification goToDock();

  /**
   * Send the robot dock location and then dock it.
   *
   * @return success or error
   */
  @TRADEService
  @Action
  Justification goHomePrimitive();

  @TRADEService
  @Action
  @Effect(
          effect={"at(?actor,unknown)"},
          type = EffectType.ALWAYS,
          infer = {"at(?actor,unknown)"}
  )
  Justification stopFollow();

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "stopFollow()", onSuspendServiceCall = "stopFollow()")
  Justification follow();

  /**
   * Close an RMS session with the robot.
   *
   * @return Justification true if successful
   */
  @Action
  @TRADEService
  Justification endSession();

  /**
   * Get the battery level of the Ava robot in percentage.
   *
   * @return double 0-100
   */
  @Action
  @TRADEService
  double getBatteryLevel();

  /**
   * Get the estimated remaining runtime of the robot in seconds.
   *
   * @return runtime seconds
   */
  @Action
  @TRADEService
  double getRuntime();

  /**
   * Get the current Z lift position of the robot.
   *
   * @return double percentage of lift 0-100
   */
  @Action
  @TRADEService
  double getZLift();

  /**
   * Set the Z lift of the robot.
   *
   * @param lift percentage to set z lift to 0-100
   */
  @TRADEService
  Justification setZLift(double lift);

  /**
   * Get the current camera tilt position of the robot.
   *
   * @return double percentage of tilt 0-100
   */
  @Action
  @TRADEService
  double getCameraTilt();


  /**
   * Set the Z lift of the robot.
   *
   * @param tilt percentage to set z lift to 0-100
   */
  @Action
  @TRADEService
  Justification setCameraTilt(double tilt);

  /**
   * Get a list of dynamic obstacles that the robot detects.
   *
   * @return Object containing obstacles list
   */
  @Action
  @TRADEService
  List<Obstacle> getDynamicObstacles();

  /**
   * Return true if robot is docked
   *
   * @return boolean
   */
  @Action
  @TRADEService
  boolean isDocked();


  @Action
  @TRADEService
  Justification lookAtObstacle(Obstacle obs);

  /**
   * Look at a vector.
   *
   * @param x        position x
   * @param y        position y
   * @param duration length of look at event
   */
  @Action
  @TRADEService
  Justification lookAt(double x, double y, double duration);

  //todo: this shouldn't live here
  @TRADEService
  @Action
  public Symbol parseTime(Symbol input, Symbol amOrPm);

  @TRADEService
  @Action
  public Integer getWifiSignalStrength();

}
