/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.operators;

import edu.tufts.hrilab.action.asl.CompiledAction;

public final class Asl {
  private Asl() { }

  public static CompiledAction compileAsl(String sourceName, @Literal String source, String entry) {
    return CompiledAction.compile(sourceName, source, entry);
  }
}
