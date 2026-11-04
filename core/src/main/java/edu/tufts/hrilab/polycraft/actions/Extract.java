/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Extract extends Active {
  protected String target;

  public Extract(String target) {
    this.target = target;
  }

  @Override
  public String getCommand() {
    return "EXTRACT_" + target;
  }
}
