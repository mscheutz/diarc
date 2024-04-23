/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Place extends Active {
  protected String target;

  public Place(String targ) {
    this.target = targ;
  }

  @Override
  public String getCommand() {
    return "PLACE " + target;
  }
}
