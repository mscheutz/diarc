/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

import java.util.Arrays;

public class SmoothMove extends Active {
  static final String[] VALID = {"W", "A", "D", "X", "Q", "E", "Z", "C"};
  protected String direction;

  public SmoothMove(String dir) {
    if (Arrays.asList(VALID).contains(dir)) {
      direction = dir;
    } else {
      throw new RuntimeException("Invalid Action Parameters");
    }
  }

  @Override
  public String getCommand() {
    return "SMOOTH_MOVE" + " " + direction;
  }

}
