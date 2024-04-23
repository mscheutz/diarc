/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */

package edu.tufts.hrilab.action.operators;

import edu.tufts.hrilab.fol.Symbol;

public final class Comparison {
  public static boolean equalsValue(Symbol a, Symbol b) {
    return a.toUntypedString().equals(b.toUntypedString());
  }

  public static boolean startsWith(String a, String b) {
    return a.startsWith(b);
  }

  public static boolean endsWith(String a, String b) {
    return a.endsWith(b);
  }

  public static boolean equalsIgnoreCase(String a, String b) {
    return a.equalsIgnoreCase(b);
  }

  public static boolean equals(Object a, Object b) {
    return a.equals(b);
  }

  public static boolean notEquals(Object a, Object b) {
    return !equals(a,b);
  }

  @OperatorSymbol("==")
  public static boolean equal(Object a, Object b) throws OperatorException {
    return (compare(a,b) == 0);
  }

  @OperatorSymbol("!=")
  public static boolean notEqual(Object a, Object b) throws OperatorException {
    return (compare(a,b) != 0);
  }

  @OperatorSymbol("gt")
  public static boolean greater(Object a, Object b) throws OperatorException {
    return (compare(a,b) > 0);
  }

  @OperatorSymbol("ge")
  public static boolean greaterOrEqual(Object a, Object b) throws OperatorException {
    return (compare(a,b) >= 0);
  }

  @OperatorSymbol("lt")
  public static boolean lower(Object a, Object b) throws OperatorException {
    return (compare(a,b) < 0);
  }

  @OperatorSymbol("le")
  public static boolean lowerOrEqual(Object a, Object b) throws OperatorException {
    return (compare(a,b) <= 0);
  }

  @SuppressWarnings("unchecked") // Sorry about that...
  private static int compare(Object a, Object b) throws OperatorException {
    // Check if objects are comparable
    if(a instanceof Comparable && b instanceof Comparable) {
      // Check if compatible types
      if(!a.getClass().isAssignableFrom(b.getClass())) {
        // Convert numbers to Double
        if (a instanceof Number && b instanceof Number) {
          a = Double.valueOf(a.toString());
          b = Double.valueOf(b.toString());
        }
        else {
          // Can't convert other types
          throw new OperatorException("Argument types are different and prevent comparison.");
        }
      }
      return ((Comparable<Object>)a).compareTo(b);
    }
    throw new OperatorException("At least one argument isn't comparable.");
  }
}
