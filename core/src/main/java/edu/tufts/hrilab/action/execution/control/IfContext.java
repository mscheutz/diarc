/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.justification.Justification;

/**
 * This represents the semantics of the IF-THEN-ELSE controls in an action script.
 * 
 * @author willie
 */
public class IfContext extends Context {
  private final ScriptParser parser;
  private ScriptParser currParserStep;

  public IfContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm,"IF");
    this.parser = parser;
    this.currParserStep = parser.getBody();
  }

  @Override
  public void setStatus(ActionStatus eStatus, Justification justification) {
    if (eStatus.isFailure() && childContexts.getNextIndex() == 1) {
      // if condition context failed, treat as if the condition returned false
      log.debug("If condition failed: " + childContexts.get(0));
    } else {
      super.setStatus(eStatus, justification);
    }
  }

  private void addNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      setupNextStep(currParserStep);
      currParserStep = currParserStep.getRest();
    }
  }

  private void skipNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      currParserStep = currParserStep.getRest();
    }
  }

  @Override
  protected void setupNextStep() {
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT: {
        int nextIndex = childContexts.getNextIndex();
        if (nextIndex == 0) {
          // add IF conditions
          addNextStep();
        } else if (nextIndex == 1) {
          if (childContexts.get(0).getLogicalValue() && !childContexts.get(0).isFailure()) {  // check IF condition
            log.debug("Child status is TRUE: " + childContexts.get(0).toString());
            addNextStep(); // add THEN
          } else {
            // do else/elseif (index 2 into childContexts), if any
            // intentionally two calls to getNextAndIncrement()
            skipNextStep(); // index 1
            addNextStep(); // index 2
          }
        } else {
          // THEN or ELSE clause, both mean IF statement is done, pop
        }
      }
      break;
      case SIMULATE:
        // simulation -> get all the child contexts
        addNextStep(); // get each child context
        break;
      case OBSERVE:
        // tmf: temporary fix, needs to be modified -> branching
        break;
      default:
        // tmf: how should performance assessment calculate which block of code to execute
        //      conditional args may be unbound?
    }
  }

  @Override
  public IfContext copy(Context newParent) {
    IfContext newIf = new IfContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newIf);
    return newIf;
  }
}
