/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.execution.Context;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Stack;

/**
 * The ActionStack stores the currently executed steps. It takes care of updating
 * the state machine and locks while hiding the low level implementation details.
 */
public class ActionStack {

  private static Logger log = LoggerFactory.getLogger(ActionStack.class);

  private ActionInterpreter ai;

  private final Stack<Context> stack = new Stack<>();

  /**
   * Constructor
   * @param a an ActionInterpreter instance
   */
  public ActionStack(ActionInterpreter a) {
    ai = a;
  }

  /**
   * Adds a step to the action stack before execution
   * @param step
   */
  public void before(Context step) {
    log.trace("ActionStack.before()");
    stack.push(step);
  }

  /**
   * Gets the next step from the action stack
   * @return next step
   */
  public Context next() {
    log.trace("ActionStack.next()");
    stack.pop();
    if (!stack.empty()) {
      return stack.peek();
    }
    return null;
  }
}
