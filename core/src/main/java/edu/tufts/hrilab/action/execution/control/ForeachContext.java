/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.util.Utilities;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * This represents the semantics of the FOR EACH control in an action script.
 * Provides a compact way to iterate over a collection.
 *
 * @author luca
 *
 * Syntax:
 * foreach (var collection) {
 * ....
 * }
 *
 * - var can be a argument (?var) or local (!var) and doesn't have to (but can) be declared at the script level
 * - collection is variable bound to a java collection
 *
 *
 * EventSpec Steps:
 * foreach condition
 * block
 * step1
 * ...
 * stepN
 * endblock
 * endfor
 */
public class ForeachContext extends ArgumentBasedContext {
  private String elementVarName;
  private String collectionVarName;
  private int index = 0;
  private final ScriptParser parser;
  private ScriptParser currParserStep;

  public ForeachContext(Context c, StateMachine sm, ScriptParser parser) {
    super(c, sm, "FOREACH");
    setupArguments(parser.getEventSpec().getInputArgs(), null);
    this.parser = parser;
  }

  /**
   * Prepare for the next iteration of the for loop. This includes cleaning up child contexts from
   * previous loop iterations.
   */
  private void initializeNextIteration() {
    // prepare for next loop iteration
    childContexts.clear(); // not strictly necessary, but prevents huge context trees from growing
    currParserStep = parser.getBody(); // prepare to re-parse, which happens in the doStep method
  }

  /**
   * Add the next child context to be executed, if one is available.
   */
  private void addNextStep() {
    // set up next step
    if (currParserStep != null && !currParserStep.isEmpty()) {
      setupNextStep(currParserStep);
      currParserStep = currParserStep.getRest();
    }
  }

  /**
   * Check the loop criteria to determine if an iteration should be performed.
   * @return
   */
  private boolean shouldExecuteLoop() {
    Collection<?> collection = (Collection) this.getArgumentValue(collectionVarName);
    if (index < collection.size()) {
      // bind element from collection to variable
      List<?> list = new ArrayList<>(collection);
      this.getArgument(elementVarName).bindDeep(list.get(index++));
      return true;
    } else {
      return false;
    }
  }

  @Override
  protected void setupNextStep() {
    switch (getExecType()) {
      case SIMULATE_ACT:
      case ACT: {
        if (shouldExecuteLoop()) {
          // setup body of for loop
          initializeNextIteration();
          addNextStep();
        }
      }
      break;
      default:
        // simulation -> get all the child contexts
        // tmf: how should execution handle number of loops? especially if condition args may not be bound
        addNextStep();
    }
  }

  /**
   * Sets up the variable for the "foreach" context.
   *
   * @param inputArgs should be 2 arguments: element and collection
   */
  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    if (inputArgs.size() == 2) {
      elementVarName = inputArgs.get(0).toString();
      collectionVarName = inputArgs.get(1).toString();

      if (Utilities.isScriptVariable(elementVarName) && Utilities.isScriptVariable(collectionVarName)) {
        if (this.getEvaluatedArgument(collectionVarName) != null) {
          if (Collection.class.isAssignableFrom(this.getArgumentType(collectionVarName))) {
            if (!this.hasArgumentInScope(elementVarName)) {
              // Create local variable if necessary/possible
              if (Utilities.isLocalVariable(elementVarName)) {
                ActionBinding element = new ActionBinding.Builder(elementVarName, Object.class).setIsLocal(true).build();
                this.arguments.put(elementVarName, element);
              } else {
                log.error("Variable " + elementVarName + " in FOREACH control element is not defined.");
                this.setStatus(ActionStatus.FAIL_SYNTAX);
              }
            }
          } else {
            log.error("FOREACH control expects collection as second argument, provided: " +
                    this.getArgumentType(collectionVarName) + ".");
            this.setStatus(ActionStatus.FAIL_SYNTAX);
          }
        } else {
          log.error("Collection " + collectionVarName + " in FOREACH control element is not defined.");
          this.setStatus(ActionStatus.FAIL_SYNTAX);
        }
      } else {
        log.error("FOREACH control expects two variables as arguments, provided: " +
                elementVarName + " and " + collectionVarName);
        this.setStatus(ActionStatus.FAIL_SYNTAX);
      }
    } else {
      log.error("FOREACH control expects two arguments, provided: " + inputArgs.size() + ".");
      this.setStatus(ActionStatus.FAIL_SYNTAX);
    }
  }

  protected void resetConcreteContext() {
    index = 0;
  }

  //FIXME: properly set up args
  @Override
  public ForeachContext copy(Context newParent) {
    ForeachContext newFor = new ForeachContext(newParent, newParent.getStateMachine(), parser);
    copyInternal(newFor);
    newFor.collectionVarName = collectionVarName;
    newFor.elementVarName = elementVarName;
    newFor.index = index;
    return newFor;
  }

}
