/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.AslTask;
import edu.tufts.hrilab.action.asl.AslValues;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.util.Utilities;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class JoinContext extends ArgumentBasedContext {
  private final List<?> inputs;
  private final List<?> outputs;
  private boolean taskJoin;
  private int resultCount;

  public JoinContext(Context c, StateMachine sm, List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    super(c, sm, "JOIN");
    inputs = List.copyOf(inputArgs);
    outputs = List.copyOf(returnArgs);
    setupArguments(inputArgs, returnArgs);
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    if (!inputArgs.isEmpty() && AslTask.class.equals(getArgumentType(inputArgs.get(0).toString()))) {
      taskJoin = true;
      if (inputArgs.size() > 2) {
        failExecution(ActionStatus.FAIL_ARGUMENTS, "Task join accepts a handle and optional timeout");
        return;
      }
      addArgument("task", AslTask.class, inputArgs.get(0), false, false);
      if (inputArgs.size() == 2) {
        String timeout = inputArgs.get(1).toString();
        if (!(Long.class.equals(AslValues.boxed(getArgumentType(timeout))) || Utilities.isLong(timeout) || Utilities.isInteger(timeout))) {
          failExecution(ActionStatus.FAIL_ARGUMENTS, "Task timeout must be Long milliseconds");
          return;
        }
        addArgument("timeout", Long.class, timeout, false, false);
      }
      for (int i = 0; i < returnArgs.size(); i++) {
        String target = returnArgs.get(i).toString();
        Class<?> type = getArgumentType(target);
        if (!Utilities.isScriptVariable(target) || type == null || i == 0 && type != ActionStatus.class) {
          failExecution(ActionStatus.FAIL_ARGUMENTS, "Task join returns ActionStatus followed by exported results");
          return;
        }
        addArgument(i == 0 ? "contextStatus" : "result" + (i - 1), type, target, false, true);
      }
      resultCount = Math.max(0, returnArgs.size() - 1);
      return;
    }
    switch (inputArgs.size()) {
      case 2:
        // timeout
        String arg = (String) inputArgs.get(1);
        if (Utilities.isScriptVariable(arg) && Long.class.equals(AslValues.boxed(getArgumentType(arg)))) {
          addArgument("timeout", Long.class, arg, false, false);
        } else if (Utilities.isLong(arg) || Utilities.isInteger(arg)) {
          addArgument("timeout", Long.class, arg, false, false);
        } else {
          log.error("[setupArguments] invalid arg. Second arg must be a timeout (Long). arg: " + arg);
          setStatus(ActionStatus.FAIL_ARGUMENTS, new ConditionJustification(false));
          return;
        }

        // don't break -- fall-through to first arg
      case 1:
        // first arg must be context id
        arg = (String) inputArgs.get(0);
        if (!Utilities.isScriptVariable(arg) || !Long.class.equals(AslValues.boxed(getArgumentType(arg)))) {
          log.error("[setupArguments] invalid arg. Must be a variable of type Long. arg: " + arg);
          setStatus(ActionStatus.FAIL_ARGUMENTS, new ConditionJustification(false));
          return;
        }
        // add new argument to this context
        addArgument("asyncId", Long.class, arg, false, false);
        break;
      default:
        log.error("[setupArguments] invalid number of args. Must pass in single arg specifying Context ID.");
        setStatus(ActionStatus.FAIL_ARGUMENTS, new ConditionJustification(false));
        return;
    }

    switch (returnArgs.size()) {
      case 0:
        return;
      case 1:
        // must be ActionStatus
        String arg = (String) returnArgs.get(0);
        if (!Utilities.isScriptVariable(arg) || !ActionStatus.class.equals(getArgumentType(arg))) {
          log.error("[setupArguments] invalid arg. Return arg must be a variable of ActionStatus type. arg: " + arg);
          setStatus(ActionStatus.FAIL_ARGUMENTS, new ConditionJustification(false));
          return;
        }

        // add new argument to this context
        addArgument("contextStatus", ActionStatus.class, arg, false, true);
        break;
      default:
        log.error("[setupArguments] invalid arg. JOIN only accepts a single (optional) return arg.");
        setStatus(ActionStatus.FAIL_ARGUMENTS, new ConditionJustification(false));
    }

  }

  @Override
  public void doStep() {
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        if (taskJoin) {
          joinTask();
          return;
        }
        long targetId = (Long) getArgumentValue("asyncId");

        // find the nearest parent action in the context tree to start search from
        Context startContext = caller;
        while (startContext != null && !startContext.isAction()) {
          startContext = startContext.getParentContext();
        }

        if (startContext == null) {
          log.error("[doStep] could not find parent action.");
          setStatus(ActionStatus.FAIL_NOTFOUND, new ConditionJustification(false));
          return;
        }

        Context targetContext = getContextHelper(targetId, startContext, this);

        if (targetContext == null) {
          log.error("[doStep] no matching context found for id: " + targetId);
          setStatus(ActionStatus.FAIL_NOTFOUND, new ConditionJustification(false));
          return;
        }

        // wait for target context to terminate
        if (hasLocalArgument("timeout")) {
          targetContext.waitForTermination((Long) getArgumentValue("timeout"));
        } else {
          targetContext.waitForTermination();
        }

        // set return ActionStatus, if passed in
        if (hasLocalArgument("contextStatus")) {
          setArgument("contextStatus", targetContext.getStatus());
        }
      default:
        break;
    }

  }

  private void joinTask() {
    try {
      AslTask task = (AslTask) getArgumentValue("task");
      if (resultCount != 0 && resultCount != task.getResultNames().size()) {
        throw new IllegalArgumentException("Task result count does not match explicit exports");
      }
      ActionStatus status = hasLocalArgument("timeout")
          ? task.await((Long) getArgumentValue("timeout")) : task.await();
      List<Object> values = new ArrayList<>();
      if (status == ActionStatus.SUCCESS) {
        for (int i = 0; i < resultCount; i++) {
          values.add(AslValues.convert(task.getResult(task.getResultNames().get(i)), getArgumentType("result" + i)));
        }
      }
      if (hasLocalArgument("contextStatus")) setArgument("contextStatus", status);
      for (int i = 0; i < values.size(); i++) setArgument("result" + i, values.get(i));
    } catch (InterruptedException exception) {
      Thread.currentThread().interrupt();
      failExecution(ActionStatus.FAIL, "Interrupted while joining ASL task");
    } catch (java.util.concurrent.ExecutionException exception) {
      failExecution(ActionStatus.FAIL, exception.toString());
    } catch (IllegalArgumentException | IllegalStateException | ArithmeticException exception) {
      failExecution(ActionStatus.FAIL_ARGUMENTS, exception.toString());
    }
  }

  @Override
  protected Context getNextStepForType() {
    return null;
  }

  /**
   * Helper method to search context tree for context with specified context ID. Starts tree search from
   * startContext (moving deeper), and ignores endContext as well as any children after and below endContext.
   *
   * @param targetId      target context id
   * @param startContext  context to start search from
   * @param ignoreContext context to ignore during search (as well as children after and below)
   * @return
   */
  private Context getContextHelper(Long targetId, Context startContext, Context ignoreContext) {
    Context matchingContext = null;
    Queue<Context> frontier = new LinkedList<>();
    frontier.add(startContext); // start at the parent
    Context currContext;
    while (matchingContext == null && !frontier.isEmpty()) {
      currContext = frontier.poll();
      if (currContext.getId().equals(targetId)) {
        matchingContext = currContext;
      } else {
        // add children to frontier (ignoring ignoreContext)
        for (Context child : currContext.getChildContexts().getChildrenContexts()) {
          if (child == ignoreContext) {
            // ignore ignoreContext and any children after it.
            break;
          } else {
            frontier.add(child);
          }
        }
      }
    }

    return matchingContext;
  }

  @Override
  public JoinContext copy(Context newParent) {
    JoinContext newJoin = new JoinContext(newParent, newParent.getStateMachine(), inputs, outputs);
    copyInternal(newJoin);
    return newJoin;
  }
}
