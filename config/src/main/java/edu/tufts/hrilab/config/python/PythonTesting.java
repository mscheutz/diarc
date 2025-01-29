package edu.tufts.hrilab.config.python;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.python.PythonWrapper;

public class PythonTesting extends DiarcConfiguration {

  @Override
  public void runConfiguration() {
    createInstance(GoalManagerComponent.class,
                    "-beliefinitfile agents/agents.pl " +
                    "-asl kinova/kinova_examples.asl ");
    String file = "examples.minimal_example";
    PythonWrapper wrapper = new PythonWrapper(file);
    wrapper.start();
  }
}