package edu.tufts.hrilab.config.python;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.moveit.MockMoveItComponent;
import edu.tufts.hrilab.moveit.MoveItComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.vision.MockVisionComponent;
import edu.tufts.hrilab.vision.VisionComponent;

public class HanoiConfig extends DiarcConfiguration {
  boolean mock = false;
  boolean mock_vis = true;

  @Override
  public void runConfiguration() {

    if(mock) {
      createInstance(MockMoveItComponent.class);
    } else {
      createInstance(MoveItComponent.class, "-config kortex.json");
    }

    if(mock_vis) {
      createInstance(MockVisionComponent.class, "sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");
    } else {
      createInstance(VisionComponent.class, "sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");
    }

    createInstance(GoalManagerComponent.class, "-beliefinitfile agents/agents.pl -asl kinova/kinova_examples.asl ");

    String file = "hanoi.hanoi_actions"; // Todo: Make this entry point for experiments
    PythonWrapper wrapper = new PythonWrapper(file, true); // Declare the wrapper as a TRADE spoke
    wrapper.start(); // Start the wrapper

  }
}