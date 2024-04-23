/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.unity.UnityAgent;
import edu.tufts.hrilab.unity.message.*;

public class UnityPR2 extends UnityAgent {

  static private Logger log = LoggerFactory.getLogger(UnityPR2.class);

  public UnityPR2() {
    super();
  }

  @Override
  protected void route (Message msg) {
    super.route(msg);
    switch (msg.action) {
      case "reset": reset(msg);
        break;
    }
  }

  /**
   * Get the stored area value of the PR2.
   */
  protected Symbol areaResponse () {
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
  @TRADEService
  @Action
  public synchronized Symbol getPR2Area () {
    return areaResponse();
  }

  /**
   * Start repairing a target
   */
  protected boolean repair (String target) {
    Message msg = new Message(this.agent, "repair", target);
    sendMessage(msg);
    return true;
  }
  @TRADEService
  @Action
  public synchronized boolean startPR2Repair (Symbol target) {
    return repair(target.toString());
  }

  /**
   * Stop repairing a target
   */
  protected boolean stopRepair (String target) {
    Message msg = new Message(this.agent, "stopRepair", target);
    sendMessage(msg);
    return true;
  }

  @TRADEService
  @Action
  public synchronized boolean stopPR2Repair (Symbol target) {
    return stopRepair(target.toString());
  }

  /**
   * Retract belief of robot position prior to starting trial. 
   */
  private void reset (Message msg) {
    String robotName = msg.agent;
    String predicateStr = "initializeTrial(" + robotName + ")";
    Predicate goal = Factory.createPredicate(predicateStr);
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class)).call(Long.class, goal);
    } catch (TRADEException e) {
      log.error("Error resetting trial", e);
    }
  }
}
