package edu.tufts.hrilab.config.kinova;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.moveit.MockMoveItComponent;
import edu.tufts.hrilab.python.PythonWrapper;

public class HanoiDemo extends DiarcConfiguration {
  @Override
  public void runConfiguration() {
    createInstance(MockMoveItComponent.class);
    // Wait for PyTRADE to start
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    String file = "hanoi.DIARC_env"; // Specify the python file you want to run, according to its package path
    PythonWrapper wrapper = new PythonWrapper(file, true); // Declare the wrapper as a TRADE spoke
    wrapper.start(); // Start the wrapper
  }
}
