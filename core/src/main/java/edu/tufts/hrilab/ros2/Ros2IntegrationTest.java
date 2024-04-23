/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2;

import edu.tufts.hrilab.diarc.DiarcComponent;

public class Ros2IntegrationTest {
  public Ros2IntegrationTest() {
    DiarcComponent.createInstance(TestGeneric.class, "");
  }

  public static void main(String[] args) {
    new Ros2IntegrationTest();
  }
}
