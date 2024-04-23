/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.db.OperatorDBEntry;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.justification.ConditionJustification;

import java.lang.reflect.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class OperatorContext extends DatabaseEntryContext<OperatorDBEntry> {

  public OperatorContext(Context parent, StateMachine sm, OperatorDBEntry operator, List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    super(parent, sm, operator);
    setupArguments(inputArgs, returnArgs);
  }

  @Override
  public void doStep() {
    log.trace("doStep in OperatorContext");

    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        executeStep();
        break;
      default:
        break;
    }
  }

  private void executeStep() {
    Collection<ActionBinding> args;
    List<Object> argBindings = new ArrayList<>();
    List<Object> varArgBindings = new ArrayList<>();
    try {
      args = collectArguments();
      ActionBinding returnArg = null;

      if (args != null) {
        for (ActionBinding arg : args) {
          if (arg.isReturn) {
            if (returnArg != null) {
              log.error("More than one return value!");
            }
            returnArg = arg;
          } else {
            if (!arg.isBound()) {
              log.error("Trying to execute operator with unbound arguments");
            }

            Object value = arg.getBindingDeep();
            if (arg.isVarArg()) {
              varArgBindings.add(value);
            } else {
              argBindings.add(value);
            }

          }
        }
      }

      if (getDBE().isVarArgs()) {
        argBindings.add(varArgBindings.toArray(new Object[varArgBindings.size()]));
      }

      Method op = getDBE().getImplementation();
      Object returnValue = op.invoke(null, argBindings.toArray());

      // bind return value
      if (returnArg == null) {
        log.debug("No designated return argument");
      } else {
        returnArg.bindDeep(returnValue);
      }

      // Update logical value of execution context if boolean return value is available
      if (returnValue != null && returnValue instanceof Boolean) {
        setLogicalValue((boolean) returnValue);
      }
    } catch (Exception e) {
      log.error("Error executing operator " + getDBE().getName() + "  with arguments: " + argBindings, e);
      this.setStatus(ActionStatus.FAIL, new ConditionJustification(false));
      setLogicalValue(false);
    }
  }

  /**
   * Private constructor for copy method.
   *
   * @param parent
   * @param sm
   * @param operator
   * @param arguments
   */
  private OperatorContext(Context parent, StateMachine sm, OperatorDBEntry operator, LinkedHashMap<String, ActionBinding> arguments) {
    super(parent, sm, operator);
    copyArguments(arguments);
  }

  @Override
  public OperatorContext copy(Context newParent) {
    OperatorContext newOpContext = new OperatorContext(newParent, newParent.stateMachine, getDBE(), this.arguments);
    copyInternal(newOpContext);
    return newOpContext;
  }
}
