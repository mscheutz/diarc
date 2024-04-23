/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.operators;

import edu.tufts.hrilab.fol.Symbol;

/**
 * This contains a bunch of miscellaneous java methods for testing the StandardJavaMethods.
 */
public class MiscJavaMethods {

  public static boolean getTrueStaticNoArgs() {
    return true;
  }

  public static boolean getFalseStaticNoArgs() {
    return false;
  }

  public static boolean getTrueStaticWithArgs(Symbol arg) {
    return true;
  }

  public static boolean getFalseStaticWithArgs(Symbol arg) {
    return false;
  }

  public boolean getTrueNoArgs() {
    return true;
  }

  public boolean getFalseNoArgs() {
    return false;
  }

  public boolean getTrueWithArgs(Symbol arg) {
    return true;
  }

  public boolean getFalseNoArgs(Symbol arg) {
    return false;
  }
}
