package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

public interface SpotInterface {
  @Action
  @TRADEService
  void setInitialLocation(Symbol refId);

  @Action
  @TRADEService
  Justification setLocationWithFiducial(Symbol refId);

  // Mostly just for testing! You should make a new action for the command you want
  @Action
  @TRADEService
  Justification command(String input);

  @Action
  @TRADEService
  Justification claim();

  @Action
  @TRADEService
  Justification release();

  @Action
  @TRADEService
  void bindLocationToCurrentPose(Symbol refId);

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
  Justification stow();

  @TRADEService
  @Action
  Justification unstow();

  @Action
  @TRADEService
  Justification carry();

  @Action
  @TRADEService
  Justification pickUpObject(Symbol refId);

  @TRADEService
  @Action
  Justification closeGripper();

  @TRADEService
  @Action
  Justification openGripper();

  @TRADEService
  @Action
  Justification goToLocationUnsafe(Symbol location);

  @TRADEService
  @Action
  boolean hasBeenLocalized();

  @TRADEService
  @Action
  Justification localize();

  @TRADEService
  @Action
  Justification detectionPose();
}
