/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

public class Reset extends Active {
  protected int seed;
  protected String location;
  protected Boolean novel;

  public Reset(String location) {
    this.maxResponseWait = 10000;
    novel = false;
    this.location = location;
  }

  public Reset(int seed, String location) {
    this.maxResponseWait = 10000;
    novel = true;
    this.seed = seed;
    this.location = location;
  }

  @Override
  public String getCommand() {
    if (novel) {
      return "RESET novel " + location + " " + Integer.toString(seed);
    } else {
      return "RESET domain " + location;
    }
  }
}