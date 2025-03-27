package edu.tufts.hrilab.config.python;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.moveit.MockMoveItComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.vision.MockVisionComponent;

public class HanoiConfig extends DiarcConfiguration {

  @Override
  public void runConfiguration() {
    createInstance(GoalManagerComponent.class, "-beliefinitfile agents/agents.pl -asl kinova/kinova_examples.asl ");
    createInstance(MockMoveItComponent.class);
    createInstance(MockVisionComponent.class, "sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");

    String file = "hanoi.hanoi_actions"; // Todo: Make this entry point for experiments
    PythonWrapper wrapper = new PythonWrapper(file, true); // Declare the wrapper as a TRADE spoke
    wrapper.start(); // Start the wrapper

  }
}