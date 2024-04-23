/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */

package edu.tufts.hrilab.action.operators;

public final class Logical {

  @OperatorSymbol("!")
  public static boolean not(Boolean a) {
    return !a;
  }

  public static boolean and(Boolean a, Boolean b) {
    return a && b;
  }

  @OperatorSymbol("||")
  public static boolean or(Boolean a, Boolean b) {
    return a || b;
  }
}
