/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class MoveBack extends Active {

  @Override
  public String getCommand() {
    return "SMOOTH_MOVE X";
  }
}
