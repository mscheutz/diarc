package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.interfaces.*;

public interface SpotInterface extends DockingInterface, MoveBaseInterface, ArmInterface, CoordinateFramesInterface, VelocityInterface, MapBuilderInterface {

  @Action
  @TRADEService
  Justification claim();

  @Action
  @TRADEService
  Justification release();

  @Action
  @TRADEService
  Justification powerOn();

  @Action
  @TRADEService
  Justification powerOff();

  @Action
  @TRADEService
  Justification stand();

  @Action
  @TRADEService
  Justification sit();

  /**
   * TODO: why won't this work from asl as a tsc: ??
   * @return
   */
  @TRADEService
  @Action
  boolean isLocalized();

  @Action
  @TRADEService
  Justification localizeWithReference(Symbol refId);

  @Action
  @TRADEService
  Justification localizeWithFiducial();

  /**
   * TODO: why won't this work from asl as a tsc: ??
   * @return
   */
   @Action
   @TRADEService
   boolean isHoldingObject();

  /**
   * TODO: remove this in favor of the graspObject method in the ArmInterface
   *
   * @param refId
   * @return
   */
  @Deprecated
  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification grabObject(Symbol refId);

  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification openDoor(Symbol doorRef, Symbol handleRef);

  @Action
  @TRADEService
  Justification goToSpecificPose(double x, double y, double z, double qx, double qy, double qz, double qw);
}
