/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

import java.util.Arrays;

public class SmoothTilt extends Active {
  static final String[] VALID = {"FORWARD", "DOWN"};
  protected String direction;

  public SmoothTilt(String dir) {
    if (Arrays.asList(VALID).contains(dir)) {
      direction = dir;
    } else {
      throw new RuntimeException("Invalid Action Parameters");
    }
  }

  @Override
  public String getCommand() {
    return "SMOOTH_TILT" + " " + direction;
  }

}

