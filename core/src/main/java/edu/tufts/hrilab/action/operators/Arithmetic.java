/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */

package edu.tufts.hrilab.action.operators;

public final class Arithmetic {

/**
 * The assign operator as implemented here doesn't work well for uninitialized arguments (null)
 * as there's no way to make sure that the value we return is of the correct type.
 * Example, this method will assign (string) "3" to an integer ActionBinding this is currently null...
 */
//public static Object assign(Object a, Object b) throws OperatorException {
//  Object value;
//  if (a != null) {
//    if (b != null) {
//      if (a.getClass() == b.getClass()) {
//        value = a;
//      } else {
//        Object convertedObject = Utilities.convertToType(b.getClass().toString(), a);
//        if (convertedObject != null) {
//          value = convertedObject;
//        } else
//          throw new OperatorException("Could not convert object of type " + a.getClass() + " to type " + b.getClass());
//      }
//    } else {
//      value = a; // TODO: value gets assigned to argument without checking for type
//    }
//  } else throw new OperatorException("Cannot assign null");
//
//  return value;
//}


  @OperatorSymbol("+")
  public static double add(Number a, Number b) {
    return a.doubleValue() + b.doubleValue();
  }

  @OperatorSymbol("++")
  public static int increment(int n) {
    return ++n;
  }

  @OperatorSymbol("-")
  public static double subtract(Number a, Number b) {
    return a.doubleValue() - b.doubleValue();
  }

  @OperatorSymbol("--")
  public static int decrement(int n) {
    return --n;
  }

  @OperatorSymbol("*")
  public static double multiply(Number a, Number b) {
    return a.doubleValue() * b.doubleValue();
  }

  @OperatorSymbol("/")
  public static double divide(Number a, Number b) {
    return a.doubleValue() / b.doubleValue();
  }

  @OperatorSymbol("%")
  public static double modulus(Number a, Number b) {
    return a.doubleValue() % b.doubleValue();
  }

  public static long round(Number a) {
    return Math.round(a.doubleValue());
  }
}
