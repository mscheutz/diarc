/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.util;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public enum Direction {
  NORTH(0),
  EAST(90),
  SOUTH(180),
  WEST(270);

  private final int degrees;
  static private Logger log = LoggerFactory.getLogger(Direction.class);

  Direction(int degrees) {
    this.degrees = degrees;
  }

  public static Direction getDirection(int degrees) {
    switch (degrees) {
      case 0:
        return NORTH;
      case 90:
        return EAST;
      case 180:
        return SOUTH;
      case 270:
        return WEST;
      default:
        log.error("Invalid direction degrees: " + degrees);
        return null;
    }
  }

  public int getDegrees() {
    return degrees;
  }
}
