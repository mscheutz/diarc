/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.operators;

public class OperatorException extends Exception {

  public OperatorException(String message) {
    super(message);
  }

  public OperatorException(Throwable cause) {
    super(cause);
  }

  public OperatorException(String message, Throwable cause) {
    super(message, cause);
  }
}
