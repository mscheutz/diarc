package edu.tufts.hrilab.config.python;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.python.PythonWrapper;

public class PythonTesting extends DiarcConfiguration {

  @Override
  public void runConfiguration() {
    createInstance(GoalManagerComponent.class,
            "-beliefinitfile agents/agents.pl " +
                    "-asl kinova/kinova_examples.asl ");
    String file = "examples.minimal_example";
    PythonWrapper wrapper = new PythonWrapper(file); //pass in true if using mac
    wrapper.start();
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("dock")).call(void.class, Factory.createSymbol(""));
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }
  }
}