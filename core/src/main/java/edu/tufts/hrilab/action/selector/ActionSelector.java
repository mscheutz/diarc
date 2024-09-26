/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author dave
 */

package edu.tufts.hrilab.action.selector;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.Justification;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public abstract class ActionSelector {

  protected static final Logger log = LoggerFactory.getLogger(ActionSelector.class);

  private static ActionSelector instance = null;
  private static Class<? extends ActionSelector> instanceType = UtilitarianActionSelector.class;
  private static Lock instanceLock = new ReentrantLock();

  /**
  * Protected to prevent instantiation except by subclasses.
  */
  protected ActionSelector() {
  }

  /**
  * Get the action selector instance.
  *
  * @return the singleton instance of an actionSelector
  */
  public static ActionSelector getInstance() {
    instanceLock.lock();
    try {
      if (instance == null) {
        try {
          instance = instanceType.newInstance();
        } catch (InstantiationException | IllegalAccessException exception) {
          log.error("couldn't instantiate action selector " + instanceType, exception);
        }
      }
    } finally {
      instanceLock.unlock();
    }

    return instance;
  }

  public static void destroyInstance() {
    instanceLock.lock();
    try {
      if (instance != null) {
        instance = null;
      }
    } finally {
      instanceLock.unlock();
    }
  }

  /**
   * Helper method to convert string form of ActionSelector type (e.g., classpath) into a
   * Class object and set the ActionSelector type.
   * @param type
   */
  public static void setActionSelectorType(String type) {
    try {
      Class clazz = Class.forName(type);
      ActionSelector.setActionSelectorType(clazz);
    } catch (ClassNotFoundException | SecurityException | IllegalArgumentException e) {
      log.error("Exception in setting ActionSelector type: " + type +
              ". Using default type instead.", e);
    }
  }

  /**
  * Set the action selector type.
  *
  * @param  type the action selector to use, a subclass of this class
  * @return      success / failure (it can't be set after instance has been constructed)
  */
  public static boolean setActionSelectorType(Class<? extends ActionSelector> type) {
    boolean result;

    instanceLock.lock();
    try {
      if (instance != null) {
        log.error("couldn't instantiate action selector, singleton already instantiated");
        result = false;
      } else {
        instanceType = type;
        result = true;
      }
    } finally {
      instanceLock.unlock();
    }

    return result;
  }

  /**
  * this can be used by action selectors (subclasses)
  * get all of the currently available actions that satisfy a goal
  *
  * @param  goal the goal to be satisfied by returned actions
  * @return      list of actions that accomplish the goal
  */
  protected static final List<ParameterizedAction> getCandidateActions(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {

    List<ParameterizedAction> candidates = new ArrayList<>();

    List<ActionDBEntry> dbLookUp = new ArrayList<>();
    if (goal.isObservation()) {
      dbLookUp.addAll(Database.getObserverDB().getObservers(goal.getActor(), goal.getPredicate()));
    } else if (goal.isAction()) {
      // add all actions based on matching action signature
      dbLookUp.addAll(Database.getActionDB().getActionsBySignature(goal.getPredicate()));
    } else {
      // add all actions based on matching post-condition
      dbLookUp.addAll(Database.getActionDB().getActionsByEffect(goal.getActor(), goal.getPredicate()));
    }

    // build the ParameterizedActions, and
    // filter them by javaType checking the actionParameters (i.e., input args)
    for (ActionDBEntry adb : dbLookUp) {
      ParameterizedAction parameterizedAction = new ParameterizedAction(adb, goal);
      if (parameterizedAction.hasValidArguments()) {
        candidates.add(parameterizedAction);
      }
    }

    // we probably want this to filter the results instead of just annotating the actions?
    // this the only reason this method takes a stateMachine and a Constraint,
    // it would be nice to put these to better use... -db
    for (ParameterizedAction action : candidates) {
      Justification justification = constraints.verifyConstraints(action, stateMachine);
      action.setJustification(justification);
    }

    return candidates;
  }

  /**
  * the main job of action selectors (subclasses) is to implement this function
  *
  * @param goal         the goal that the selected action should accomplish
  * @param constraints  a returned goal should satisfy any constraints
  * @param stateMachine in case the action selector cares about the state
  * @return             an action that accomplishes goal, satisfies constraints
  */
  public abstract ParameterizedAction selectActionForGoal(Goal goal, ActionConstraints constraints, StateMachine stateMachine);


  public ActionDBEntry selectActionForCommand(Symbol actor, String command, List<String> inputArguments, ActionConstraints constraints, StateMachine stateMachine){

    return selectActionForGoal(new Goal(actor, Factory.createPredicate(command,inputArguments.toArray(new String[0]))),constraints,stateMachine).getEntry();

  }
}

