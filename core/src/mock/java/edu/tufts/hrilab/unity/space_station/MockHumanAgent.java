package edu.tufts.hrilab.unity.space_station;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class MockHumanAgent extends DiarcComponent {

  private String agent;

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("agent").hasArg().argName("agent").desc("Set the agent name.").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("agent")) {
      agent = cmdLine.getOptionValue("agent");
    } else {
      log.error("Agent name is required. Should be either \"player1\" or \"player2\".");
    }
  }

  /**
   * Set human workload in belief.
   *
   * @param level 0 = low workload, 1 = high workload
   */
  @TRADEService
  public void setWorkload(Integer level) {
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
