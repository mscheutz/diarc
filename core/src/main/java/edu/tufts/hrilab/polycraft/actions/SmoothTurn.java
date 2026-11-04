/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class SmoothTurn extends Active {
  protected int angle;

  public SmoothTurn(int angle) {
    if (angle % 15 == 0) {
      this.angle = angle;
    } else {
      throw new RuntimeException("Invalid Action Parameters");
    }
  }

  @Override
  public String getCommand() {
    return "SMOOTH_TURN" + " " + angle;
  }

}
