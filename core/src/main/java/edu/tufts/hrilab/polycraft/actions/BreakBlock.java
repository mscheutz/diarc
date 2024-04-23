/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class BreakBlock extends Active {

  public BreakBlock() {
  }

  @Override
  public String getCommand() {
    return "BREAK_BLOCK";
  }

}