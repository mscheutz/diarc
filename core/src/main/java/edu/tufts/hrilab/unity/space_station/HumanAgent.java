package edu.tufts.hrilab.unity.space_station;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.unity.UnityAgent;
import edu.tufts.hrilab.unity.message.Message;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class HumanAgent extends UnityAgent {
  static private Logger log = LoggerFactory.getLogger(HumanAgent.class);

  public HumanAgent() {
    super();
    setWorkload(0);
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

  private void workload (Message msg) {
    int level = Integer.parseInt(msg.arguments.get(0));
    setWorkload(level);
  }

  private void setWorkload (int level) {
    //workload(player1,0)
    Predicate workloadRetraction = Factory.createPredicate("workload", agent, "X");
    Predicate workloadBelief = Factory.createPredicate("workload", agent, String.valueOf(level));
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief")).call(Object.class, workloadRetraction);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, workloadBelief);
      log.debug("Set workload for " + agent + " to " + level);
    } catch (TRADEException ex) {
      log.error("Error retracting or asserting workload belief", ex);
    }
  }
}
