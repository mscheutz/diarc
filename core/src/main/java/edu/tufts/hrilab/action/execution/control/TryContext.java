/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.justification.Justification;

/**
 * This represents the semantics of the TRY control in an action script.
 */
public class TryContext extends Context {

  /**
   * The action status of the action that was caught.
   */
  private ActionStatus statusToCatch;

  /**
   * Intercepted exit status to propagate up context tree after finally block has been executed.
   */
  private ActionStatus statusToPropagate;

  /**
   * Intercepted exit justification to propagate up context tree after finally block has been executed.
   */
  private Justification justificationToPropagate;

  /**
   * Internal state of this TryContext.
   */
  private TryContextState tryState;

  /**
   * Helper enum to track the internal state of the TryContext.
   */
  private enum TryContextState {
    NORMAL,
    SHOULD_CATCH,
    SHOULD_EXECUTE_FINALLY,
    DONE
  }

  private final ScriptParser parser;
  private ScriptParser currParserStep;

  /**
   * Constructor.
   *
   * @param c
   */
  public TryContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "TRY");
    this.parser = parser;
    init();
  }

  /**
   * Initialize TryContext state. Also used in resetStatus so that TryContext
   * can be re-used.
   */
  private void init() {
    statusToCatch = null;
    statusToPropagate = null;
    justificationToPropagate = null;
    tryState = TryContextState.NORMAL;
    currParserStep = parser.getBody();
  }

  @Override
  public void resetConcreteContext() {
    init();
  }

  /**
   * Exit the current event. Note that this is always called from the child
   * event, setting its execution status before it returns.
   * <p>
   * Special handling of error in try context: do not transmit error to parent context, instead handle the
   * error within the context by jumping to the catch block.
   * If an exit happens in the catch block, resume standard behavior by transmitting error to parent context.
   *
   * @param eStatus the execution status of the currently running event
   */
  @Override
  public void setStatus(ActionStatus eStatus, Justification justification) {
    if (!eStatus.isTerminated()) {
      super.setStatus(eStatus, justification);
    } else if (tryState == TryContextState.NORMAL && canCatch(eStatus, justification)) {
      tryState = TryContextState.SHOULD_CATCH;
      statusToCatch = eStatus;
    } else if (tryState != TryContextState.DONE) {
      // temporarily intercept exit to run finally block
      // exit is resumed after finally block is executed in DONE case of switch statement in getNextStepForType
      tryState = TryContextState.SHOULD_EXECUTE_FINALLY;
      statusToPropagate = eStatus;
      justificationToPropagate = justification;
    } else {
      super.setStatus(eStatus, justification);
    }
  }

  @Override
  protected void setupNextStep() {
    switch (getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        setupStepForState();
        break;
    }
  }

  private void setupStepForState() {

    switch (tryState) {
      case NORMAL:
        // if next step is not CATCH or FINALLY add it as the next step to execute
        ControlFactory.Control controlType = ControlFactory.Control.fromString(currParserStep.getCommand());
        if (controlType != ControlFactory.Control.CATCH && controlType != ControlFactory.Control.FINALLY) {
          setupNextStep(currParserStep);
          currParserStep = currParserStep.getRest();
        } else {
          // done with normal execution, move on to finally block (if one exists)
          tryState = TryContextState.SHOULD_EXECUTE_FINALLY;
          setupStepForState();
        }
        break;

      case SHOULD_CATCH:
        // appropriate catch context has already been added as next step to execute in canCatch method

        // set next state
        tryState = TryContextState.SHOULD_EXECUTE_FINALLY;
        break;

      case SHOULD_EXECUTE_FINALLY:
        if (!hasFinallyContext()) {
          // if no finally context, explicitly recurse so that DONE case is executed
          // this is necessary because this method won't be called again if the next
          // step is null (i.e., no finally child to execute)
          tryState = TryContextState.DONE;
          setupStepForState();
        } else {
          tryState = TryContextState.DONE;
        }
        break;

      case DONE:
        // need to enter DONE state before ending try context (i.e., returning null) in case
        // an exit status/justification needs to be propagated up context tree
        if (statusToPropagate != null) {
          super.setStatus(statusToPropagate, justificationToPropagate);
        }
        break;

      default:
        log.error("[setupStepForState] in default TryContextState with state: " + tryState);
    }

  }

  /**
   * Check if any of this try's CatchContexts can catch the ActionStatus. If so, set next step to be
   * the matching CatchContext.
   *
   * @param status status to catch
   * @param justification justification for status
   * @return
   */
  private boolean canCatch(ActionStatus status, Justification justification) {
    // can only catch failures
    if (!status.isFailure()) {
      return false;
    }

    // find applicable catch context (if any) and set it as the next child context to execute
    while (currParserStep != null && !currParserStep.isEmpty()) {
      ControlFactory.Control controlType = ControlFactory.Control.fromString(currParserStep.getCommand());
      if (controlType == ControlFactory.Control.CATCH) {
        // add catch context to children
        CatchContext catchContext = (CatchContext) setupNextStep(currParserStep);

        // if can catch all failures (i.e., statusToCatch is null) or specific failure
        if (catchContext.getStatusToCatch() == null || catchContext.getStatusToCatch() == status) {
          // set index to point to catch context
          childContexts.setNextIndex(childContexts.size() - 1); // TODO: is this necessary?
          catchContext.transmitErrorInfo(justification);
          return true;
        } else {
          // remove catch context so that it isn't executed
          childContexts.remove(catchContext);
        }
      }
      currParserStep = currParserStep.getRest();
    }

    // no matching catch context found to handle failure
    return false;
  }

  /**
   * Search through possible child contexts and set the finally context as
   * the next child to execute if one exists.
   *
   * @return
   */
  private boolean hasFinallyContext() {
    // find finally in the parse and add it to the children
    while (currParserStep != null && !currParserStep.isEmpty()) {
      ControlFactory.Control controlType = ControlFactory.Control.fromString(currParserStep.getCommand());
      if (controlType == ControlFactory.Control.FINALLY) {
        setupNextStep(currParserStep);
        // set index to point to finally context
        childContexts.setNextIndex(childContexts.size() - 1); // TODO: is this necessary?
        return true;
      }
      currParserStep = currParserStep.getRest();
    }

    // no finally context
    return false;
  }

  @Override
  public TryContext copy(Context newParent) {
    TryContext newTry = new TryContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newTry);
    newTry.statusToCatch = statusToCatch;
    newTry.justificationToPropagate = justificationToPropagate;
    newTry.statusToPropagate = statusToPropagate;
    newTry.tryState = tryState;
    return newTry;
  }
}
