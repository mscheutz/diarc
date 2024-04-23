/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.dialogue;


import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.util.Map;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Question Answering
 */

public class Answer {
  Lock responseLock;
  Condition responseCondition;
  Map<Variable, Symbol> answerBindings = null;

  public Answer() {
    this.responseLock = new ReentrantLock();
    this.responseCondition = responseLock.newCondition();
  }

  public Lock getResponseLock() {
    return responseLock;
  }

  public Condition getResponseCondition() {
    return responseCondition;
  }

  public void setAnswerBindings(Map<Variable, Symbol> bindings) {
    this.answerBindings = bindings;
  }

  public Map<Variable, Symbol> getAnswerBindings() {
    return answerBindings;
  }
}