/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.AndJustification;

import edu.tufts.hrilab.action.util.Utilities;

import java.util.*;

/**
 * The Exit control can be used to end the execution of a script.
 * An ActionStatus can be specified to set the exit status.
 * If the ActionStatus is a failure status, a list of Predicates (or java Collections)
 * can be specified as arguments and will be used as failConditions to explain
 * the reason behind the exit.
 * Usage example:
 * {@code <control>exit FAIL not(safe) danger(ahead)</control>}
 * will exit with a FAIL status and set the two above predicates as fail conditions.
 * Note that Collections can also be used as arguments, e.g. a set of predicates.
 */
public class ExitContext extends ArgumentBasedContext {
  private String exitStatus = null;

  public ExitContext(Context c, StateMachine sm, List<? extends Object> inputArgs) {
    super(c, sm, "EXIT");
    setupArguments(inputArgs, null);
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    for (int i = 0; i < inputArgs.size(); ++i) {
      boolean isAS = false;

      // Get exit status
      if (exitStatus == null) {
        if (Utilities.isScriptVariable(inputArgs.get(i).toString()) && getArgumentType(inputArgs.get(i).toString()).equals(ActionStatus.class)) {
          exitStatus = inputArgs.get(i).toString(); // Keep variable
          isAS = true;
        } else if (ActionStatus.fromString(inputArgs.get(0).toString()) != null) {
          exitStatus = inputArgs.get(0).toString(); // Keep status (in string representation)
          isAS = true;
        }
      }

      // Get fail conds
      if (!isAS) {
        String name = "failCond" + i;
        Object argValue = inputArgs.get(i).toString();
        String argValueStr = argValue.toString();
        Class<?> argClass;

        // check if failure condition arg is a script variable
        ActionBinding argBinding = null;
        if (hasArgumentInScope(argValueStr)) {
          argBinding = getArgument(argValueStr);
        }

        // if failure condition is a script variable, bind the local arg to the ActionBinding, otherwise it must
        // be a String representation of a Predicate
        if (argBinding != null) {
          argClass = getArgumentType(argValueStr);
          argValue = argBinding;
        } else {
          argClass = Predicate.class;
        }

        if (argClass != Predicate.class && !Collection.class.isAssignableFrom(argClass) && argClass != Justification.class) {
          log.error("[setupArguments] Cannot handle argument of type: " + argValue.getClass());
          continue;
        }

        addArgument(name, argClass, argValue, false, false);
      }
    }
  }

  @Override
  public void doStep() {
    log.debug("[doStep] Ending execution (ExitContext)");

    // build justification
    Justification justification;
    ActionStatus status;
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        if (!arguments.isEmpty()) {
          AndJustification andJustification = new AndJustification();
          for (String role : arguments.keySet()) {
            Object value = getArgumentValue(role); //evaluates unevaluated args
            if (value instanceof Predicate) {
              Predicate boundValue = this.bindPredicate((Predicate) value);//evaluates unevaluated args
              ConditionJustification tmpJustification = new ConditionJustification(false, Factory.createNegatedPredicate(boundValue));
              andJustification.addJustification(tmpJustification);
            } else if (value instanceof Collection) {
              AndJustification tmpJustification = new AndJustification();
              for (Object p : (Collection) value) {
                if (p instanceof Predicate) {
                  tmpJustification.addJustification(new ConditionJustification(false, Factory.createNegatedPredicate((Predicate) p)));
                } else if (p instanceof Justification) {
                  tmpJustification.addJustification((Justification) p);
                  //add middle step
                  tmpJustification.setStep(((Justification) p).getStep());
                } else {
                  log.error("[doStep] Cannot handle Collection of argument type: " + p.getClass());
                }
              }
              andJustification.addJustification(tmpJustification);
              andJustification.setStep(tmpJustification.getStep());
            } else if (value instanceof Justification) {
              andJustification.addJustification((Justification) value);
              andJustification.setStep(((Justification) value).getStep());
            } else {
              log.error("[doStep] Cannot handle argument type: " + value.getClass());
            }
          }
          justification = andJustification;
        } else {
          justification = new ConditionJustification(false);
        }

        // Get exit status
        status = ActionStatus.FAIL; //Default

        if (exitStatus != null) {
          if (Utilities.isScriptVariable(exitStatus)) {
            status = (ActionStatus) getArgumentValue(exitStatus);
          } else {
            status = ActionStatus.fromString(exitStatus);
          }
        }
        this.setStatus(status, justification);
        break;
      default:
        break; // don't execute when simulating
    }
  }

  @Override
  protected Context getNextStepForType() {
    return null;
  }

  @Override
  public ExitContext copy(Context newParent) {
    ExitContext newExit = new ExitContext(newParent, newParent.getStateMachine(), new ArrayList<>());
    newExit.exitStatus = exitStatus;
    copyInternal(newExit);
    return newExit;
  }
}
