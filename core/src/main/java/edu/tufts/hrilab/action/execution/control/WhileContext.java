/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;

/**
 * This represents the semantics of the WHILE-DO controls in an action script.
 *
 * EventSpec Steps:
 * while
 * condition
 * do (block)
 * step1
 * ...
 * stepN
 * endwhile (endblock)
 *
 * @author willie
 */
public class WhileContext extends Context {
  private final ScriptParser parser;
  private ScriptParser currParserStep;
  private ControlFactory.Control executionState = ControlFactory.Control.WHILE;

  public WhileContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "WHILE");
    this.parser = parser;
  }

  private void initializeNextIteration() {
    // prepare for next loop iteration
    childContexts.clear(); // not strictly necessary, but prevents huge context trees from growing
    currParserStep = parser.getBody(); // prepare to re-parse, which happens in the doStep method
  }

  private void addNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      setupNextStep(currParserStep);
      currParserStep = currParserStep.getRest();
    }
  }

  @Override
  protected void setupNextStep() {
    switch (getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        switch (executionState) {
          case WHILE: { // execute WHILE condition
            log.debug("Setup condition of the while loop.");
            initializeNextIteration();
            addNextStep();
            executionState = ControlFactory.Control.DO; // next step
            break;
          }
          case DO: { // execute DO context (i.e., WHILE body)
            executionState = ControlFactory.Control.WHILE;  // next step
            if (this.childContexts.getCurrent().getLogicalValue()) { // WHILE condition is true
              log.debug("condition true, setting up body");
              addNextStep();
              break;
            } else {
              // WHILE condition is false, exit loop (i.e., don't add next step)
            }
          }
        }
    }
  }

  @Override
  public WhileContext copy(Context newParent) {
    WhileContext newWhile = new WhileContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newWhile);
    return newWhile;
  }

}
