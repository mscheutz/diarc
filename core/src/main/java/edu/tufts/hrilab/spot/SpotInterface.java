package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.*;
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

  @TRADEService
  @Action
  boolean isLocalized();

  @Action
  @TRADEService
  void localizeWithReference(Symbol refId);

  @Action
  @TRADEService
  void localizeWithFiducial();

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
}
