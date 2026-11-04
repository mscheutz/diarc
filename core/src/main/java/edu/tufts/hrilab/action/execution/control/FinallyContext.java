/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;


public class FinallyContext extends Context {
  private final ScriptParser parser;
  private ScriptParser currParserStep;

  public FinallyContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "FINALLY");
    this.parser = parser;
    this.currParserStep = parser.getBody();
  }

  @Override
  protected void setupNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      setupNextStep(currParserStep);
      currParserStep = currParserStep.getRest();
    }
  }

  @Override
  public FinallyContext copy(Context newParent) {
    FinallyContext newFinal = new FinallyContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newFinal);
    return newFinal;
  }
}
