/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.util.Utilities;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class JoinContext extends ArgumentBasedContext {

  public JoinContext(Context c, StateMachine sm, List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    super(c, sm, "JOIN");
    setupArguments(inputArgs, returnArgs);
  }

  private JoinContext(Context c, StateMachine sm) {
    super(c, sm, "JOIN");
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    switch (inputArgs.size()) {
      case 2:
        // timeout
        String arg = (String) inputArgs.get(1);
        if (Utilities.isScriptVariable(arg) && getArgumentType(arg).equals(Long.class)) {
          addArgument("timeout", Long.class, arg, false, false);
        } else if (Utilities.isLong(arg) || Utilities.isInteger(arg)) {
          addArgument("timeout", Long.class, arg, false, false);
        } else {
          log.error("[setupArguments] invalid arg. Second arg must be a timeout (Long). arg: " + arg);
          return;
        }

        // don't break -- fall-through to first arg
      case 1:
        // first arg must be context id
        arg = (String) inputArgs.get(0);
        if (!Utilities.isScriptVariable(arg) || !getArgumentType(arg).equals(Long.class)) {
          log.error("[setupArguments] invalid arg. Must be a variable of type Long. arg: " + arg);
          return;
        }
        // add new argument to this context
        addArgument("asyncId", Long.class, arg, false, false);
        break;
      default:
        log.error("[setupArguments] invalid number of args. Must pass in single arg specifying Context ID.");
    }

    switch (returnArgs.size()) {
      case 0:
        return;
      case 1:
        // must be ActionStatus
        String arg = (String) returnArgs.get(0);
        if (!Utilities.isScriptVariable(arg) || !getArgumentType(arg).equals(ActionStatus.class)) {
          log.error("[setupArguments] invalid arg. Return arg must be a variable of ActionStatus type. arg: " + arg);
          return;
        }

        // add new argument to this context
        addArgument("contextStatus", ActionStatus.class, arg, false, true);
        break;
      default:
        log.error("[setupArguments] invalid arg. JOIN only accepts a single (optional) return arg.");
    }

  }

  @Override
  public void doStep() {
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        long targetId = (Long) getArgumentValue("asyncId");

        // find the nearest parent action in the context tree to start search from
        Context startContext = caller;
        while (startContext != null && !startContext.isAction()) {
          startContext = startContext.getParentContext();
        }

        if (startContext == null) {
          log.error("[doStep] could not find parent action.");
          return;
        }

        Context targetContext = getContextHelper(targetId, startContext, this);

        if (targetContext == null) {
          log.error("[doStep] no matching context found for id: " + targetId);
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

  //FIXME: properly set up args
  @Override
  public JoinContext copy(Context newParent) {
    JoinContext newJoin = new JoinContext(newParent, newParent.getStateMachine(), new ArrayList<>(), new ArrayList<>());
    copyInternal(newJoin);
    return newJoin;
  }
}
