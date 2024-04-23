/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.navigation;

import edu.tufts.hrilab.action.ActionStatus;

/**
 *
 * @author evankrause
 */
public abstract class NavigationAction implements Runnable {

  public static enum ActionType {

    NONE, MOVETHROUGH, MOVETO, MOVETOXY, MOVEDIST, TIMEMOVE, MOVETOREL, TURNTO, TURNTOPOINT, TURNDIST, TIMETURN,
    STOP, SETVELS, SETTV, SETRV, FOLLOWWALL, TRAVERSE, APPROACHVISREF, APPROACHVISCOLOR
  }

  protected final ActionType type;
  private ActionStatus stat;
  private boolean runFlag;
  protected final long ID;
  protected double[] dargs;
  protected String[] stargs;
  protected long[] largs;

  public NavigationAction(ActionType t) {
    type = t;
    stat = ActionStatus.UNKNOWN;
    runFlag = true;
    ID = System.currentTimeMillis();
  }

  public NavigationAction(ActionType t, double[] args) {
    type = t;
    stat = ActionStatus.UNKNOWN;
    runFlag = true;
    ID = System.currentTimeMillis();

    dargs = new double[args.length];
    for (int i = 0; i < args.length; ++i) {
      dargs[i] = args[i];
    }
  }

  public NavigationAction(ActionType t, String[] args) {
    type = t;
    stat = ActionStatus.UNKNOWN;
    runFlag = true;
    ID = System.currentTimeMillis();

    stargs = new String[args.length];
    for (int i = 0; i < args.length; ++i) {
      stargs[i] = args[i];
    }
  }

  public NavigationAction(ActionType t, long[] args) {
    type = t;
    stat = ActionStatus.UNKNOWN;
    runFlag = true;
    ID = System.currentTimeMillis();
    largs = new long[args.length];
    for (int i = 0; i < args.length; ++i) {
      largs[i] = args[i];
    }
  }

  /**
   * Get action status.
   *
   * @return
   */
  public synchronized ActionStatus getStatus() {
    return stat;
  }

  /**
   * Set action status.
   *
   */
  public synchronized void setStatus(ActionStatus s) {
    stat = s;
  }

  /**
   * ID of action.
   *
   * @return - timestamp when action was created.
   */
  public synchronized long getID() {
    return ID;
  }

  /**
   * Set the run flag status. Use this to end processing of a run loop.
   *
   * @param flag
   */
  public synchronized void setRunFlag(boolean flag) {
    runFlag = flag;
  }

  /**
   * Run flag status. Automatically set to true on object instantiation,
   * and will return true even if action is not currently running. Run methods
   * that continually loop should use this method to know if the loop should terminate.
   *
   * @return boolean - runFlag
   */
  public synchronized boolean getRunFlag() {
    return runFlag;
  }

  /**
   * Main run method to be implemented.
   * This is the method ran by the NavigationActionManager once
   * a NavigationAction instance is submitted to the manager via
   * initiateAction.
   */
  public abstract void run();

  /**
   * Main action cancellation method to be implemented.
   * This method cancels the processing of the run method if
   * this instance is currently being ran by a NavigationActionManager.
   *
   * @return - if Action was currently being ran by manager.
   */
  public abstract boolean cancel();
}
