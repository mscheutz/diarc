package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

public interface SpotInterface {

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
  boolean hasBeenLocalized();

  @TRADEService
  @Action
  Justification localize();

  @Action
  @TRADEService
  void setInitialLocation(Symbol refId);

  @Action
  @TRADEService
  Justification setLocationWithFiducial(Symbol refId);

  @Action
  @TRADEService
  void bindLocationToCurrentPose(Symbol refId);

  /**
   * TODO: remove this in favor of the graspObject method in the ArmCInterface
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
