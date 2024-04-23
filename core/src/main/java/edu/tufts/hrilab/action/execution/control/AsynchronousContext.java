/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Variable;

import java.util.ArrayList;
import java.util.List;

public class AsynchronousContext extends ArgumentBasedContext {
  private ScriptParser parser;
  private ScriptParser currParserStep;

  public AsynchronousContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "ASYNC");
    setupArguments(parser.getEventSpec().getInputArgs(), parser.getEventSpec().getReturnArgs());
    this.parser = parser;
    this.currParserStep = parser.getBody();
  }

  @Override
  public boolean isAsynchronous() {
    return true;
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    if (!inputArgs.isEmpty()) {
      log.error("ASYNC does not accept input arguments: " + inputArgs);
    }

    if (returnArgs.size() == 1) {
      if (!Utilities.isScriptVariable(returnArgs.get(0).toString())) {
        log.error("[setupArguments] invalid arg. Must be a variable. arg: " + returnArgs.get(0));
        return;
      }
      // add new argument to this context
      addArgument("asyncId", Long.class, returnArgs.get(0), false, true);
      setArgument("asyncId", getId());
    } else if (returnArgs.size() > 1) {
      log.error("[setupArguments] invalid number of args.");
    }

    // zero return args is acceptable, no additional work needed
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
  public AsynchronousContext copy(Context newParent) {
    List<Object> newArgs = new ArrayList<>();
    addArgument("asyncId", Long.class, getId(), true, false);
    setArgument("asyncId", getId());
    Variable newArg = Factory.createVariable(getId().toString());
    newArgs.add(newArg);
    return new AsynchronousContext(newParent, newParent.getStateMachine(), parser);
  }
}
