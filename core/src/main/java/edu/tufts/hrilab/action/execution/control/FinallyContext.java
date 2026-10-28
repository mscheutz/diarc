/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import java.util.List;


public class FinallyContext extends ArgumentBasedContext {
  private final ScriptParser parser;
  private ScriptParser currParserStep;

  public FinallyContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "FINALLY");
    setupArguments(List.of(), List.of());
    this.parser = parser;
    this.currParserStep = parser.getBody();
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    // lexical scope only
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
    newFinal.copyArguments(this.arguments);
    return newFinal;
  }
}
