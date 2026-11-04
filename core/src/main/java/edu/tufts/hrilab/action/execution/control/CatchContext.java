/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.execution.Context;

import java.util.ArrayList;
import java.util.List;

public class CatchContext extends ArgumentBasedContext {
  private ActionStatus statusToCatch = null;
  private String failVarName = null;

  private final ScriptParser parser;
  private ScriptParser currParserStep;

  public CatchContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, (parser.getEventSpec().getInputArgs().size() > 0) ? "CATCH " + parser.getEventSpec().getInputArgs().get(0) : "CATCH (default)");
    setupArguments(parser.getEventSpec().getInputArgs(), null);
    this.parser = parser;
    this.currParserStep = parser.getBody();
  }

  /**
   * Transmits error information to the catch context.
   * To add support for a new failure state, add an new case in the switch statement,
   * changing the type cast to the appropriate type for the error status. See setupArguments().
   *
   * @param details details about the error
   */
  protected void transmitErrorInfo(Justification details) {
    if (failVarName != null && !failVarName.isEmpty() && details != null) {
      this.setArgument(failVarName, details);
    }
  }

  /**
   * Sets up the arguments for the catch context.
   * To add support for a new failure state, add a new case in the switch statement,
   * creating an ActionBinding for the appropriate type (error message type). See transmitErrorInfo().
   *
   * @param inputArgs
   * @param returnArgs
   */
  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    if (inputArgs.size() == 1) {
      if (inputArgs.get(0).toString().startsWith("!")) {
        failVarName = inputArgs.get(0).toString();
        ActionBinding failureAB = new ActionBinding.Builder(failVarName, Justification.class).setIsLocal(true).build();
        this.arguments.put(failVarName, failureAB);
      } else {
        statusToCatch = ActionStatus.fromString(inputArgs.get(0).toString());
      }
    } else if (inputArgs.size() == 2) {
      statusToCatch = ActionStatus.fromString(inputArgs.get(0).toString());

      failVarName = inputArgs.get(1).toString();
      ActionBinding failureAB = new ActionBinding.Builder(failVarName, Justification.class).setIsLocal(true).build();
      this.arguments.put(failVarName, failureAB);
    } else if (inputArgs.size() >= 3) {
      log.error("Invalid catch format: " + inputArgs);
    }
  }

  @Override
  protected void setupNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      setupNextStep(currParserStep);
      currParserStep = currParserStep.getRest();
    }
  }

  /**
   * Get ActionStatus that this CatchContext is capable of catching.
   *
   * @return
   */
  public ActionStatus getStatusToCatch() {
    return statusToCatch;
  }

  @Override
  public CatchContext copy(Context newParent) {
    CatchContext newCatch = new CatchContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newCatch);
    return newCatch;
  }
}
