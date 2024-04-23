/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Craft extends Active {
  protected String arguments;

  public Craft(String arguments) {
    this.arguments = arguments;
  }

  @Override
  public String getCommand() {
    return "CRAFT " + arguments;
  }
}

