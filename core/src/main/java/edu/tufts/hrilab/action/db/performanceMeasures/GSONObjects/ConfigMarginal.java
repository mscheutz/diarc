/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects;

public class ConfigMarginal {
  private String name;
  private int holds;

  public String getName() {
    return name;
  }

  public int getHolds () {
    return holds;
  }

  public ConfigMarginal(String name, int holds) {
    this.name = name;
    this.holds = holds;
  }
}
