package edu.tufts.hrilab.config.vision;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.vision.RosVisionComponent;

public class RosVisionConfig extends DiarcConfiguration {
  @Override
  public void runConfiguration() {
    createInstance(RosVisionComponent.class, "-vis -vis_frame camera_color_optical_frame");
    createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture rsense_ros.xml");
  }

  public static void main(String[] args) {
    RosVisionConfig config = new RosVisionConfig();
    config.runConfiguration();
  }
}
