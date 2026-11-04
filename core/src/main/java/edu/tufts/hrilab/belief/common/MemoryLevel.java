/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief.common;

public enum MemoryLevel {
  UNIVERSAL(0),
  EPISODIC(1),
  WORKING(2);

  private int level;

  MemoryLevel(int level) {
    this.level = level;
  }

  public boolean includesLevel(MemoryLevel level) {
    return level.level <= this.level;
  }

}
