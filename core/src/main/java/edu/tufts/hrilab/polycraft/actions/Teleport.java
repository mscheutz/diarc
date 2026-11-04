/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Teleport extends Active {
  protected String arguments;

  public Teleport(String arguments) {
    this.arguments = arguments;
  }

  @Override
  public String getCommand() {
    return "TP_TO " + arguments;
  }
}
