/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Start extends Active {

  public Start() {
    this.maxResponseWait = 10000;
  }

  @Override
  public String getCommand() {
    return "START";
  }
}
