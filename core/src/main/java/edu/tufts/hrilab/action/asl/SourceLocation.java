/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import java.io.Serializable;

/** Source coordinates are one-based; origin identifies translated source, if any. */
public record SourceLocation(String source, int line, int column, String origin) implements Serializable {
  @Override
  public String toString() {
    return source + ":" + line + ":" + column + (origin == null ? "" : " [" + origin + "]");
  }
}
