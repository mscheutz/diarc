/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Delete extends Active {
  protected String target;

  public Delete(String targ) {
    this.target = targ;
  }

  @Override
  public String getCommand() {
    return "DELETE " + target + " 1";
  }
}
