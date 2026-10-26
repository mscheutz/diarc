package edu.tufts.hrilab.config.python;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.moveit.MockMoveItComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.vision.MockVisionComponent;

public class HanoiConfig extends DiarcConfiguration {
  boolean mock = true;
  boolean mock_vis = true;

  @Override
  public void runConfiguration() {

    if(mock) {
      createInstance(MockMoveItComponent.class);
    } else {
//      createInstance(edu.tufts.hrilab.moveit.MoveItComponent.class, "-config kortex.json");
    }

    if(mock_vis) {
      createInstance(MockVisionComponent.class, "sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");
    } else {
//      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");
    }

    createInstance(GoalManagerComponent.class, "-beliefinitfile agents/agents.pl -asl kinova/kinova_examples.asl ");

    String file = "diarc.hanoi.hanoi_actions"; // Todo: Make this entry point for experiments
//    String file = "fast_neurosym_il.experiments_diffusion_diarc"; // Todo: Make this entry point for experiments
    PythonWrapper wrapper = new PythonWrapper(file, true); // Declare the wrapper as a TRADE spoke
    wrapper.start(); // Start the wrapper

  }
}