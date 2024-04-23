/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.util;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayDeque;
import java.util.Queue;

/**
 * Utility methods for exploring, searching, analyzing, etc the Context tree.
 */
public class ContextUtils {

  static private Logger log = LoggerFactory.getLogger(ContextUtils.class);

  /**
   * Use the step predicate to find the matching action, or closest parent to the matching context, in the context tree.
   * <p>
   * TODO: there's an interesting question here about what depth in the context
   * tree should be reported:
   * (1) the deepest matching action which could be a primitive,
   * (2) the deepest script, since primitives can't really be modified dynamically,
   * (3) the top-most action, or (4) something in between.
   * <p>
   * Currently, (2) is being returned if a script is found, otherwise fall back to (1). That is, once
   * the matching context has been found, return it's closest parent that is an ActionContext and is a script.
   * If no script can be found, return the closest parent that is an ActionContext (which will be a primitive).
   *
   * @param rootContext - Context to start searching from. Will only search rootContext and deeper.
   * @param stepPredicate - step of context trying to match
   * @param actionStatus - status of context
   * @return
   */
  static public ActionContext getActionContext(Context rootContext, Term stepPredicate, ActionStatus actionStatus) {
    if (stepPredicate == null) {
      log.debug("Step predicate is null. Returning null.");
      return null;
    }

    // search context tree for matching step (breadth first)
    Context matchingContext = getMatchingContext(rootContext, stepPredicate, actionStatus);
    if (matchingContext == null) {
      log.error("Could not find matching context in context tree: " + stepPredicate);
      return null;
    }

    // search up the tree to find the closest ActionContext script to the matching Context.
    Context matchingActionContext = matchingContext;
    while (matchingActionContext != null && !isActionScript(matchingActionContext)) {
      matchingActionContext = matchingActionContext.getParentContext();
    }

    if (matchingActionContext == null) {
      log.debug("No action script in the context tree. Finding any ActionContext.");
      matchingActionContext = matchingContext;
      while (matchingActionContext != null && !matchingActionContext.isAction()) {
        matchingActionContext = matchingActionContext.getParentContext();
      }
    }

    if (matchingActionContext == null || !(matchingActionContext instanceof ActionContext)) {
      log.warn("Found matching context in context tree but no parent action script for: " + stepPredicate);
      return null;
    }

    // guaranteed to be of type ActionContext
    return (ActionContext) matchingActionContext;
  }

  /**
   * Find the Context in the context tree that matches the stepPredicate. The context getSignatureInPredicateForm
   * must exactly match the stepPredicate. Will return null when no match is found.
   * @param rootContext - Context to start searching from. Will only search rootContext and deeper.
   * @param stepPredicate - step of context trying to match
   * @param actionStatus - status of context
   * @return
   */
  static public Context getMatchingContext(Context rootContext, Term stepPredicate, ActionStatus actionStatus) {
    return getMatchingContext(rootContext, stepPredicate, actionStatus ,null);
  }

  /**
   * Find the Context in the context tree that matches the stepPredicate. The context getSignatureInPredicateForm
   * must exactly match the stepPredicate. Will return null when no match is found.
   * @param rootContext - Context to start searching from. Will only search rootContext and deeper.
   * @param stepPredicate - step of context trying to match
   * @param actionStatus - status of context
   * @param locationReference - reference step used to locate correct step
   * @return
   */
  static public Context getMatchingContext(Context rootContext, Term stepPredicate, ActionStatus actionStatus, Term locationReference) {
    // search context tree for matching step (breadth first)
    Queue<Context> frontier = new ArrayDeque<>();
    frontier.add(rootContext);
    Context matchingContext = null;
    while (!frontier.isEmpty()) {
      Context currContext = frontier.poll();
      // check matching signature
      if (currContext.getSignatureInPredicateForm().equals(stepPredicate)) {
        // check matching action status
        if (actionStatus != null) {
          if (currContext.getStatus() == actionStatus) {
            matchingContext = currContext;
            break;
          }
          frontier.addAll(currContext.getChildContexts().getChildrenContexts());
        } else {
          matchingContext = currContext;
          break;
        }
      } else {
        frontier.addAll(currContext.getChildContexts().getChildrenContexts());
      }
    }
    return matchingContext;
  }

  /**
   * Helper method to check if Context is an action script context.
   *
   * @param context
   * @return
   */
  static public boolean isActionScript(Context context) {
    return context.isAction() && ((ActionContext) context).isScript();
  }

  /**
   * get the context which caused the failure
   * @param context failed context
   * @return the child which caused the failure, or null if no failure found
   */
  static public Context getFailureContext(Context context) {
    if (context.isFailure()) {
      for (Context child : context.getChildContexts().getChildrenContexts()) {
        if (child.isFailure()) {
          if (child.causedFailure()) {
            return child;
          } else {
            return getFailureContext(child);
          }
        }
      }
    }
    return null;
  }

}
