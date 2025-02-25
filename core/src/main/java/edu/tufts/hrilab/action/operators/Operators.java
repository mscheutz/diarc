/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.operators;

public class Operators {
  private final static Class[] operatorClasses = {
          edu.tufts.hrilab.action.operators.Arithmetic.class,
          edu.tufts.hrilab.action.operators.Collections.class,
          edu.tufts.hrilab.action.operators.Comparison.class,
          edu.tufts.hrilab.action.operators.Logical.class,
          edu.tufts.hrilab.action.operators.Misc.class,
          edu.tufts.hrilab.action.operators.StandardJavaMethods.class,
          edu.tufts.hrilab.action.operators.FOL.class,
          edu.tufts.hrilab.action.PerformanceAssessment.class,
          edu.tufts.hrilab.fol.Factory.class
  };

  public static Class[] getClasses() {
    return operatorClasses;
  }
}
