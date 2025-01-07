/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import java.util.List;

import ai.thinkingrobots.trade.TRADEServiceConstraints;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.unity.message.*;
import edu.tufts.hrilab.unity.space_station.SpaceStationObservers;

public class UnityPR2Component extends UnityAgentComponent implements UnityPR2Interface {
  private SpaceStationObservers observers;

  @Override
  public void init() {
    observers = new SpaceStationObservers();
    observers.registerWithTRADE(this.getMyGroups());
  }

  @Override
  protected void route(Message msg) {
    super.route(msg);
    switch (msg.action) {
      case "reset":
        reset(msg);
        break;
      case "end":
        end(msg);
        break;
    }
  }

  /**
   * Get the stored area value of the PR2.
   */
  protected Symbol areaResponse() {
    Message msg = new Message(this.agent, "area");
    List<String> res = null;
    try {
      res = getResponse(msg);
    } catch (Exception e) {
      log.error("Error sending " + msg.action + " message", e);
    }
    if (res == null || res.size() != 1) {
      log.error("Invalid response from PR2 re: " + msg.action);
      return null;
    }
    return Factory.createSymbol(res.get(0));
  }

  @Override
  public synchronized Symbol getPR2Area() {
    return areaResponse();
  }

  /**
   * Start repairing a target
   */
  protected boolean repair(String target) {
    Message msg = new Message(this.agent, "repair", target);
    sendMessage(msg);
    return true;
  }

  @Override
  public synchronized boolean startPR2Repair(Symbol target) {
    return repair(target.toString());
  }

  /**
   * Stop repairing a target
   */
  protected boolean stopRepair(String target) {
    Message msg = new Message(this.agent, "stopRepair", target);
    sendMessage(msg);
    return true;
  }

  @Override
  public synchronized boolean stopPR2Repair(Symbol target) {
    return stopRepair(target.toString());
  }

  /**
   * Retract belief of robot position prior to starting trial.
   */
  private void reset(Message msg) {
    String robotName = msg.agent;
    String predicateStr = "initializeTrial(" + robotName + ")";
    Predicate goal = Factory.createPredicate(predicateStr);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class)).call(Long.class, goal);
    } catch (TRADEException e) {
      log.error("Error resetting trial", e);
    }
  }

  /**
   * Issue endTrial() action to cancel all active goals and stop the robot.
   */
  private void end(Message msg) {
    String robotName = msg.agent;
    String predicateStr = "endTrial(" + robotName + ")";
    Predicate goal = Factory.createPredicate(predicateStr);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class)).call(Long.class, goal);
    } catch (TRADEException e) {
      log.error("Error ending trial", e);
    }
  }
}
