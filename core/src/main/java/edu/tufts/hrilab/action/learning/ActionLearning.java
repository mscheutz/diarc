/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Top level class in charge of maintaining action learning, including the current learning state
 * and other paused learning states (a paused learning state is different from a parent learning state),
 * ...
 */
public class ActionLearning {
  private static final Logger log = LoggerFactory.getLogger(ActionLearning.class);
  private ExecutionManager executionManager;
  private boolean executeAction = false;
  private Set<Predicate> ignoreActionSet = new HashSet<>();
  private Queue<Goal> goalQueue = new LinkedBlockingQueue<>();
  private Lock goalQueueEmptyLock = new ReentrantLock();
  private Condition goalQueueNotEmpty = goalQueueEmptyLock.newCondition();
  private Thread goalQueueManager;
  private Goal currentGoal = null;
  private LearningState currentLearningState = null;
  /**
   * Current action learning status. Only set this field via setLearningStatus method.
   */
  private ActionLearningStatus learningStatus = ActionLearningStatus.NONE;
  private Map<String, LearningState> pausedLearningStates = new HashMap<>();
  private ActionLearningGui learningGui;

  /**
   * Learning status lock.
   */
  private Lock statusLock = new ReentrantLock();
  /**
   * Status lock condition that is signalled when the status changes.
   */
  private Condition statusCondition = statusLock.newCondition();

  /**
   * @param em Execution Manager instance to run executeWhileLearning and simulation goals
   */
  public ActionLearning(ExecutionManager em, boolean useGui) {
    executionManager = em;

    if (useGui) {
      log.debug("creating gui for action learning");
      learningGui = new ActionLearningGui();
    }

    //TODO:EW: I don't like this mechanism to determine what is a taught step vs a current command.
    //  No flexibility, hard to maintain
    //set which actions/goals to ignore and pass through to be executed during learning
    //ignoreActionSet.add(Factory.createPredicate("sayText(?actor,?text)"));
    ignoreActionSet.add(Factory.createPredicate("handled(?actor,?utterance)"));
    ignoreActionSet.add(Factory.createPredicate("modifyAction(?actor,?agent,?action,?modification,?location)"));
    ignoreActionSet.add(Factory.createPredicate("learnAction(?actor,?agent,?action)"));
    ignoreActionSet.add(Factory.createPredicate("endActionLearning(?actor,?agent,?action)"));
    ignoreActionSet.add(Factory.createPredicate("pauseActionLearning(?actor,?agent,?action)"));
    ignoreActionSet.add(Factory.createPredicate("resumeActionLearning(?actor,?agent,?action)"));
    ignoreActionSet.add(Factory.createPredicate("cancelActionLearning(?actor,?agent,?action)"));
    ignoreActionSet.add(Factory.createPredicate("changeLearningExecution(?actor,?agent,?status)"));
    ignoreActionSet.add(Factory.createPredicate("endLearningAssembleScript(?actor,?action)"));
    ignoreActionSet.add(Factory.createPredicate("freeze(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("endFreeze(?actor)"));

    ignoreActionSet.add(Factory.createPredicate("cancelSystemGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("suspendSystemGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("resumeSystemGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelSystemGoals(?actor,?agent)"));
    ignoreActionSet.add(Factory.createPredicate("suspendSystemGoals(?actor,?agent)"));
    ignoreActionSet.add(Factory.createPredicate("resumeSystemGoals(?actor,?agent)"));
    ignoreActionSet.add(Factory.createPredicate("cancelAllCurrentGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelAllPendingGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelAllActiveGoals(?actor)"));
    ignoreActionSet.add(Factory.createPredicate("cancelPendingGoalByIndex(?actor)"));
  }

  public void startQueuingGoals() {
    goalQueueManager = new Thread(() -> {
      while (learningStatus.equals(ActionLearningStatus.ACTIVE)) {
        if (!goalQueue.isEmpty()) {
          Goal g = goalQueue.remove();
          if (shouldExecute()) {
            //Temporarily add to ignore set so it is executed and we don't get stuck in a loop here
            ignoreActionSet.add(g.getPredicate());
            g.setIsLearningGoal(true);
            g = executionManager.submitGoal(g, ExecutionType.ACT);
            GoalStatus goalStatus = executionManager.joinOnGoal(g.getId());
            ignoreActionSet.remove(g.getPredicate());

            if (goalStatus == GoalStatus.SUCCEEDED) {
              currentLearningState.createEventSpec(g);
            } else {
              log.error("not adding goal to learned action because it failed: " + g);
              //TODO:brad:how should the rest of the system find out about this
            }
          } else {
            g.setStatus(GoalStatus.SUCCEEDED);
            currentLearningState.createEventSpec(g);
          }

        } else {
          goalQueueEmptyLock.lock();
          try {
            goalQueueNotEmpty.await();
          } catch (Exception e) {
            log.error("[startQueuingGoals]", e);
          } finally {
            goalQueueEmptyLock.unlock();
          }
        }
      }
    });
    //TODO:brad end this thread somewhere?
    goalQueueManager.start();
  }

  /**
   * Helper method to lock when setting the action learning status. All code that sets this field should use
   * this method instead of setting it directly.
   *
   * @param status
   */
  private void setLearningStatus(ActionLearningStatus status) {
    try {
      statusLock.lock();
      learningStatus = status;
      statusCondition.signalAll();
    } finally {
      statusLock.unlock();
    }
  }

  public void waitForActionLearningStart(Predicate newAction) {
    try {
      statusLock.lock();
      while (learningStatus != ActionLearningStatus.ACTIVE && currentLearningState != null && !newAction.getName().equalsIgnoreCase(currentLearningState.getName())) {
        try {
          statusCondition.await();
        } catch (InterruptedException e) {
          log.warn("[waitForActionLearningStart] Interrupted while waiting for learning to start.", e);
        }
      }
    } finally {
      statusLock.unlock();
    }
  }

  //Start learning new action
  public boolean learnAction(Predicate newAction) {
    log.debug("[learnAction] " + newAction);
    String actName = newAction.getName();

    // check to see if we are currently learning something
    if (learningStatus == ActionLearningStatus.ACTIVE) {
      try {
        if (!(currentLearningState.getName().equalsIgnoreCase(actName))) {
          currentLearningState = new LearningState(newAction, currentLearningState);
        } else {
          log.debug("Already learning new action: " + newAction);
        }
      } catch (NullPointerException e) {
        log.debug("[learnAction] supposedly already learning but LS is null... something is wrong");
      }
    } else {
      currentLearningState = new LearningState(newAction);
    }
    // TODO: spawn observers based on situation and task relevant info
    if (learningGui != null) {
      learningGui.setLearningState(currentLearningState);
    }
    setLearningStatus(ActionLearningStatus.ACTIVE);
    startQueuingGoals();

     //to keep the learning goal active while learning, wait here until status is no longer active
      try {
        statusLock.lock();
        while (learningStatus == ActionLearningStatus.ACTIVE) {
          try {
            statusCondition.await();
          } catch (InterruptedException e) {
            log.warn("Interrupted while waiting for learning to finish.", e);
          }
        }
      } finally {
        statusLock.unlock();
      }

    return true;
  }

  //Cleanly finish learning, if parent learning state then resume learning
  public boolean endActionLearning(Predicate newAction) {
    log.debug("[endActionLearning] " + newAction);
    boolean currentState;
    LearningState learningState;
    // check to see if the predicate action is same as action being learned
    if (newAction.getName().equalsIgnoreCase(currentLearningState.getName())) {
      learningState = currentLearningState;
      currentState = true;
    } else {
      learningState = pausedLearningStates.get(newAction.getName());
      currentState = false;
      if (learningState == null) {
        log.error("[endActionLearning] called for action with no learning state: {}", newAction);
        return false;
      }
    }

    log.debug("Generating new ActionDBEntry");
    learningState.generateActionDBEntry();
    LearningState previousLearningState = learningState.getPreviousLearningState();
    if (previousLearningState != null) {
      log.debug("restoring previous action: " + learningState.getPreviousLearningState().getName());
      if (currentState) {
        currentLearningState = previousLearningState;
        startQueuingGoals();
        if (learningGui != null) {
          learningGui.setLearningState(currentLearningState);
        }
      } else {
        pausedLearningStates.put(newAction.getName(), previousLearningState);
      }
    } else {
      log.debug("ending learning for action {}", newAction);
      if (currentState) {
        setLearningStatus(ActionLearningStatus.END);
        currentLearningState = null;
        executeAction = false;
        if (learningGui != null) {
          learningGui.clear();
        }
      } else {
        pausedLearningStates.remove(newAction.getName());
      }
    }
    return true;
  }

  public boolean cancelActionLearning(Predicate newAction) {
    log.debug("[cancelActionLearning] " + newAction);
    if (newAction.getName().equalsIgnoreCase(currentLearningState.getName())) {
      setLearningStatus(ActionLearningStatus.CANCEL);
      currentLearningState = null;
      setLearningStatus(ActionLearningStatus.NONE);
      if (learningGui != null) {
        learningGui.clear();
      }
    } else {
      LearningState learningState = pausedLearningStates.remove(newAction.getName());
      if (learningState == null) {
        log.error("[cancelActionLearning] called for action with no learning state: {}", newAction);
        return false;
      }
    }
    return true;
  }

  //Pauses learning of action -> stores in paused map -> restored in future
  public boolean pauseActionLearning(Predicate newAction) {
    log.debug("[pauseActionLearning] " + newAction);
    if (newAction.getName().equalsIgnoreCase(currentLearningState.getName())) {
      pausedLearningStates.put(newAction.getName(), currentLearningState);
      currentLearningState = null;
      setLearningStatus(ActionLearningStatus.NONE);
    } else {
      log.error("[pauseActionLearning] provided action is not equal to action currently being learned");
      return false;
    }
    return true;
  }

  //Resume the paused action, pop from map
  public boolean resumeActionLearning(Predicate newAction) {
    log.debug("[resumeActionLearning] " + newAction);
    LearningState resumedLearningState = pausedLearningStates.get(newAction.getName());
    if (resumedLearningState != null) {
      if (currentLearningState != null) {
        pausedLearningStates.put(currentLearningState.getName(), currentLearningState);
      }
      currentLearningState = resumedLearningState;
      pausedLearningStates.remove(newAction.getName());
      setLearningStatus(ActionLearningStatus.ACTIVE);
      startQueuingGoals();
      // TODO: restart / create observers
    } else {
      //tmf: agent should convey that it hasn't started learning something.
      // TODO: create interpreter to run say text to respond accordingly
      log.error("cannot resume " + newAction.getName() + " because haven't started learning before");
      setLearningStatus(ActionLearningStatus.NONE);
    }
    return true;
  }

  public boolean shouldIgnore(Goal goal) {
    return ignoreActionSet.stream().anyMatch(a -> goal.getPredicate().instanceOf(a));
  }

  /**
   * check to see if the goal should be added to the action being learned and if so add it.
   *
   * @param goal goal to add
   */

  public void addGoal(Goal goal) {
    goalQueue.add(goal);

    goalQueueEmptyLock.lock();
    try {
      goalQueueNotEmpty.signal();
    } catch (Exception e) {
      log.error("[addGoal] ", e);
    } finally {
      goalQueueEmptyLock.unlock();
    }
  }

  public ActionLearningStatus getLearningStatus() {
    return learningStatus;
  }

  public void changeLearningExecution(Symbol status) {
    executeAction = status.getName().equals("execute");
  }

  public boolean shouldExecute() {
    return executeAction;
  }

  //TODO: cancelTeachingByModification
  //  Remove new entry that was formed but canceled halfway through teaching template
//Brad: moved from ActionModification.java which no longer exists.

  /**
   modifyAction(actor,action, modification, location)
   @param action : predicate containing action and arguments:
   arguments are required for grounding of new action/goal onto learning state
   actionName(actor, arg0, arg1, ... argN)
   predicate containing new action predicate and old task predicate
   like(newAction(arg0, arg1 ... argN), existingAction(arg0, arg1 ... argN)
   @param modification : information regarding the modification to make
   : type(actionStep, actionStep)
   : type(actionStep)
   : type(type, type) //TODO:brad is this actually valid?
   : type: insert, delete, replace
   : insert(action_predicate)
   : delete(action_predicate)
   : replace(new_action_predicate, old_action_predicate)
   @param location: reference in action script
   : relation(actionStep, actionStep)
   : relation(actionStep)
   : relation: and/before/after/condition/effect/argument
   : before(action_predicate)
   : after(action_predicate)
   : and(relation,relation))
   : condition()
   : effect()
   : argument()
   */

  public void modifyAction(Predicate action, Predicate modification, Predicate location) {
    /*
      check the name of the action predicate, if it is some value then, the inner
      predicates are actions themselves and denote a different action will be created

      currently:
        if the first argument of the action predicate is a predicate, then
          the system is learning a new action based on a known action
    */
    LearningState learningState;
    ActionDBEntry existingEntry = null;
    ActionDBEntry knownEntry;
    List<Symbol> args = action.getArgs();
    if (action.getName().equals("like")) {
      action = (Predicate) args.get(0);
      Predicate knownAction = (Predicate) args.get(1);
      knownEntry = getDBEntry(knownAction);
    } else {
      knownEntry = getDBEntry(action);
      existingEntry = knownEntry;
    }

    //TODO:brad:revisit the usage of learningState here?
    // get/create learning state
    if (knownEntry == null) {
      // this is the case when modifying an action while learning the action
      learningState = this.currentLearningState;
    } else {
      // modifying an action that already exists
      learningState = new LearningState(knownEntry, action);
    }

    // finally, modify the action
    modifyAction(modification, location, learningState);
    if (existingEntry != null) {
      Database.getActionDB().removeAction(existingEntry);
    }
    learningState.generateActionDBEntry();
  }

  private void modifyAction(Predicate modification, Predicate location, LearningState ls) {
    String operation = modification.getName();

    switch (location.getName()) {
      case "condition": {
        //TODO: handle disjunction;
        if (operation.equals("insert")) {
          ls.addCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(0), ConditionType.fromString(location.get(0).toString())));
        } else if (operation.equals("delete")) {
          ls.removeCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(0), ConditionType.fromString(location.get(0).toString())));
        } else { //TODO:brad: should we explicitly handle replace here? and fail otherwise?
          ls.removeCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(1), ConditionType.fromString(location.get(0).toString())));
          ls.addCondition(new edu.tufts.hrilab.action.Condition((Predicate) modification.get(0), ConditionType.fromString(location.get(0).toString())));
        }
        break;
      }
      case "effect": {
        if (operation.equals("insert")) {
          // TODO: fix comparison, cannot just add because no equals in effect
          ls.addEffect(new Effect((Predicate) modification.get(0), EffectType.fromString(location.get(0).toString())));
        } else if (operation.equals("delete")) {
          ls.removeEffect(new Effect((Predicate) modification.get(0), EffectType.fromString(location.get(0).toString())));
        } else {
          ls.removeEffect(new Effect((Predicate) modification.get(1), EffectType.fromString(location.get(0).toString())));
          ls.addEffect(new Effect((Predicate) modification.get(0), EffectType.fromString(location.get(0).toString())));
        }
        break;
      }
      case "argument":
        //TODO
        if (operation.equals("insert")) {
          log.warn("[modifyAction] inserting arguments not supported");
        } else if (operation.equals("delete")) {
          log.warn("[modifyAction] deleting arguments not supported");
        }
        break;
      default:  // modify action steps
        int modLocation;
        switch (operation) {
          case "removeLocalVar":
            ls.removeEventSpecsForLocalVariable(modification.get(0));
            break;
          case "insert":  // add action to script
            if (location.getName().equals("none")) {
               ls.addEventSpec(ls.getEventSpecs().size(), (Predicate) modification.get(0));
              log.warn("[modifyAction] inserting step at end of action");
              return;
            }
            modLocation = ls.getIndexOfConstraint(location);
            if (modLocation >= 0) {
              // TODO: this probably should not be done by extracting predicate, but idk
              ls.addEventSpec(modLocation, (Predicate) modification.get(0));
            } else {
              log.warn("[modifyAction] no event spec for " + location);
              return;
            }
            break;
          case "remove":  // remove action from script
            if (location.getName().equals("none")) {
              modLocation = ls.getEventStepLocation((Predicate) modification.get(0));
            } else {
              modLocation = ls.getIndexGivenConstraints(location, (Predicate) modification.get(0));
            }
            if (modLocation >= 0) {
              ls.removeEventSpec(modLocation);
            } else {
              log.warn("[modifyAction] no event spec for " + modification + " with reference step " + location);
              return;
            }
            break;
          case "replace":  // replace action within script
            Predicate oldStep = (Predicate) (modification.get(1));
            if (location.getName().equals("none")) {
              modLocation = ls.getEventStepLocation(oldStep);
            } else {
              modLocation = ls.getIndexGivenConstraints(location, oldStep);
            }
            if (modLocation >= 0) {
              // TODO: this probably should not be done by extracting predicate, but idk
              String newLv = ls.addEventSpec(modLocation, (Predicate) modification.get(0));
              ls.removeEventSpec(modLocation + 1, newLv);

            } else {
              log.warn("[modifyAction] no event spec for " + modification + " with reference step " + modLocation);
              return;
            }
            break;
          default:
            log.error("invalid action modification operation: " + operation + " in " + modification + " at " + location);
            break;
        }
        break;
    }
  }

  private ActionDBEntry getDBEntry(Predicate task) {
    List<ActionDBEntry> possibleEntries = Database.getActionDB().getActionsBySignature(task);
    if (possibleEntries.isEmpty()) {
      return null;
    }
    return possibleEntries.get(0);
  }

  enum ModType {
    INSERT,
    DELETE,
    REPLACE;

    static boolean isModType(String type) {
      ModType val = Utilities.strToEnum(ModType.class, type);
      return val != null;
    }

    static ModType fromString(String typeStr) {
      if (typeStr != null) {
        for (ModType t : ModType.values()) {
          if (typeStr.equalsIgnoreCase(t.name())) {
            return t;
          }
        }
      }
      log.error("Unknown modification type: " + typeStr);
      return null;
    }
  }

  enum RelType {
    before,
    after;

    static boolean isRelType(String relation) {
      RelType val = Utilities.strToEnum(RelType.class, relation);
      return val != null;
    }

    static RelType fromString(String typeStr) {
      if (typeStr != null) {
        for (RelType t : RelType.values()) {
          if (typeStr.equalsIgnoreCase(t.name())) {
            return t;
          }
        }
      }
      log.error("Unknown relation type: " + typeStr);
      return null;
    }
  }

}
