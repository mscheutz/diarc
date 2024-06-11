/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.kinova;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.moveit.MoveItComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class KinovaDemo extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(KinovaDemo.class);

  // start the configuration
  public static void main(String[] args) {
    DiarcConfiguration config = new KinovaDemo();
    config.runConfiguration();
  }

  @Override
  public void runConfiguration() {
    // optional VisionComponent if your kortex arm has a realsense camera on the wrist and you have the ros_kortex_vision package running
    // createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-cameraFrame camera_color_frame -capture kinova.xml -refs refs/visionRefs.json");

    createInstance(MoveItComponent.class, "-config kortex.json");
    createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class,
               "-goal startKinova(self) " +
                    "-beliefinitfile agents/agents.pl " +
                    "-asl kinova/kinova_examples.asl ");

  }
}
