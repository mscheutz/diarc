package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.spot.msgs.GazeRequestMessage;
import edu.tufts.hrilab.spot.msgs.GazeResponseMessage;

import java.util.concurrent.ExecutionException;

public interface MockSpotInterface {
  @Action
  @TRADEService
  public void setInitialLocation(Symbol refId);

  @Action
  @TRADEService
  public Justification setLocationWithFiducial(Symbol refId);

  // Mostly just for testing! You should make a new action for the command you want
  @Action
  @TRADEService
  public Justification command(String input);

  @Action
  @TRADEService
  public Justification claim();

  @Action
  @TRADEService
  public Justification release();

  @Action
  @TRADEService
  public void bindLocationToCurrentPose(Symbol refId);

  @Action
  @TRADEService
  public Justification dock(int id);

  @Action
  @TRADEService
  public Justification powerOn();

  @Action
  @TRADEService
  public Justification powerOff();


  @Action
  @TRADEService
  public Justification stand();

  @Action
  @TRADEService
  public Justification sit();

  @TRADEService
  @Action
  public Justification stow();

  @TRADEService
  @Action
  public Justification unstow();

  @Action
  @TRADEService
  public Justification carry();

  @Action
  @TRADEService
  public Justification pickUpObject(Symbol refId);

  @TRADEService
  @Action
  public Justification closeGripper();

  @TRADEService
  @Action
  public Justification openGripper();

  @TRADEService
  @Action
  public Justification goToLocationUnsafe(Symbol location);

  @TRADEService
  @Action
  public boolean hasBeenLocalized();

  @TRADEService
  @Action
  public Justification localize();

  @TRADEService
  @Action
  public Justification detectionPose();

}
