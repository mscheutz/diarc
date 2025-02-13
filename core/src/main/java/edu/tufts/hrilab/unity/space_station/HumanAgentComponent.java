/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.unity.UnityAgentComponent;
import edu.tufts.hrilab.unity.message.Message;

public class HumanAgentComponent extends UnityAgentComponent {

  public HumanAgentComponent() {
    super();
  }

  @Override
  protected void route (Message msg) {
    super.route(msg);
    switch (msg.action) {
      case "workload" :
        workload(msg);
        break;
    }
  }

  @Override
  protected void init () {
    setWorkload(0);
  }

  private void workload (Message msg) {
    int level = Integer.parseInt(msg.arguments.get(0));
    log.info("Workload received: {}", level);
    setWorkload(level);
  }

  private void setWorkload (int level) {
    //workload(player1,0)
    Term workloadRetraction = Factory.createPredicate("workload", agent, "X");
    Term workloadBelief = Factory.createPredicate("workload", agent, String.valueOf(level));
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief").argTypes(Term.class)).call(Object.class, workloadRetraction);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief").argTypes(Term.class)).call(Object.class, workloadBelief);
      log.debug("Set workload for " + agent + " to " + level);
    } catch (TRADEException ex) {
      log.error("Error retracting or asserting workload belief", ex);
    }
  }
}
