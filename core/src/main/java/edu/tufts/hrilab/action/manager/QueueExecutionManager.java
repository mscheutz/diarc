/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PendingGoal;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.Util;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class QueueExecutionManager extends ExecutionManager {
  private static final Logger log = LoggerFactory.getLogger(QueueExecutionManager.class);

  //Current active goal (i.e. has an associated running interpreter) per agent which has been processed through the
  // queue. Each agent may have multiple active goals present at any time, but this is the one which determines the
  // availability of an agent
  private Map<Symbol,Goal> agentGoals;

  public QueueExecutionManager() {
    super();
  }

  protected void init(StateMachine sm, RootContext rootContext, String priorityFile, Collection<String> groups) {
    super.init(sm, rootContext, priorityFile,groups);
    agentGoals = new HashMap<>();
    for (Symbol agent : agentTeams.keySet()) {
      agentGoals.put(agent, null);
    }
  }

  @Override
  protected void onActiveGoalUpdated(Goal g, GoalStatus status, UpdateType updateType) {
    //If goal skipped the queue, don't track as active goal
    if (g.getPriorityTier() == PriorityTier.SKIPPENDING) {// || currentlyLearning()) {
      return;
    }

    //New goal was added as active, update current goal for that agent and all children
    Symbol actor = getUntypedSymbol(g.getActor());
    if (updateType == UpdateType.ADDED) {
      if (!agentGoals.containsKey(actor)) {
        log.warn("[onActiveGoalUpdated] received goal with actor not present in group hierarchy: " + actor);
      } else {
        log.info("[onActiveGoalUpdated] adding agentGoal {} {} {}", g, status, updateType);
        agentGoals.put(actor, g);
      }
    }

    //Notify UI of new goal
    notifyUIActiveGoalUpdated(g, status, updateType);

    //Assign as many pending goals as possible (in order of priority) with resources freed up by this active goal
    // completing
    if (status.isTerminated() && !pendingGoals.isEmpty() && consumedResources(g)) {
      activateValidPendingGoals();
    }
  }

  @Override
  protected void handleConflictingLowerPriorityGoal(Goal g, Set<Resource> necessaryResources) {
    //QueueEM: Simply leave in the pending queue rather than rejecting
  }

  //TODO: extend definition of resources past just whole agents/teams
  @Override
  protected Set<Resource> getRequiredResourcesForGoal(Goal goal) {
    Set<Resource> resourceSet = new HashSet<>();

    //Iterate over all gathered agents and pool agent wide resources
    for (Symbol agent : getDescendants(goal.getActor())) {
      //Until we have a better definition of resources per agentTeam, currently
      // assuming any goal locks the whole agentTeam.
      //If the goal skips the queue, it has no resource dependencies.
      // Otherwise, takes the agent-wide lock.
      if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
        //resourceSet.add(agentTeams.get(agent).getResource(agent));
        resourceSet.addAll(agentTeams.get(agent).getResources());
      }
    }

    return resourceSet;
  }


  //TODO: the below method can definitely be implemented generally and moved to the EM/GMImpl, it will just take some
  //  time to track down all usages and update them accordingly.
  //Necessary subclass specific services?
  /**
   * Remove a pending goal from the queue based on index before it is transferred to active
   */
  @Action
  @TRADEService
  public boolean cancelGoalInQueueIndex(int index) {
    synchronized (pendingGoalsLock) {
      log.info("[cancelGoalInQueueIndex] removing goal in queue with index: " + index);
      log.debug("[cancelGoalInQueue] goalQueue pre: " + pendingGoals);
      if (index < pendingGoals.size()) {
        Iterator<PendingGoal> pendingGoalsIterator = pendingGoals.iterator();
        int i = 0;
        while (i < index) {
          pendingGoalsIterator.next();
          i++;
        }
        PendingGoal entry = pendingGoalsIterator.next();
        cancelGoal(entry.getGoal().getId());
        //Removing the goal itself and clearing any associated data is handled in goalCanceled
        return true;
      }
      log.warn("[cancelGoalInQueueIndex] Queue is of length: " + pendingGoals.size() + ", cannot remove goal with index: " + index);
    }

    return false;
  }

  //TODO: Can we move these methods to the base EM or use those which exist in core.asl?
  //      What does the 'current goal' mean in the general sense? If there are
  //      N active goals, do we pick the oldest, most recent, one at random?
  @Action
  @TRADEService
  public long cancelCurrentGoal() {
    return cancelCurrentGoal(rootAgent);
  }

  //TODO: return value doesn't make sense anymore if acting on all children
  @Action
  @TRADEService
  public long cancelCurrentGoal(Symbol agent) {
    log.debug("[cancelCurrentGoal] in method");
    agent = getUntypedSymbol(agent);
    long goalId = getSystemGid(agent);
    if (goalId != -1) {
      cancelGoal(goalId);
    }

    AgentTeam agentTeam = agentTeams.get(agent);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        cancelCurrentGoal(member);
      }
    }

    return -1;
  }

  @Action
  @TRADEService
  public long suspendCurrentGoal() {
    return suspendCurrentGoal(rootAgent);
  }

  //TODO: return value doesn't make sense anymore if acting on all children
  @Action
  @TRADEService
  public long suspendCurrentGoal(Symbol agent) {
    log.debug("[suspendCurrentGoal] in method");
    agent = getUntypedSymbol(agent);
    long goalId = getSystemGid(agent);
    if (goalId != -1) {
      suspendGoal(goalId);
    }

    AgentTeam agentTeam = agentTeams.get(agent);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        suspendCurrentGoal(member);
      }
    }

    return -1;
  }

  @Action
  @TRADEService
  public long resumeCurrentGoal() {
    return resumeCurrentGoal(rootAgent);
  }

  //TODO: return value doesn't make sense anymore if acting on all children
  @Action
  @TRADEService
  public long resumeCurrentGoal(Symbol agent) {
    log.debug("[resumeCurrentGoal] in method");
    agent = getUntypedSymbol(agent);
    long goalId = getSystemGid(agent);
    if (goalId != -1) {
      resumeGoal(goalId);
    }

    AgentTeam agentTeam = agentTeams.get(agent);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        resumeCurrentGoal(member);
      }
    }

    return -1;
  }

  private Long getSystemGid(Symbol actor) {
    if (agentGoals.get(actor) != null) {
      return agentGoals.get(actor).getId();
    } else {
      return -1L;
    }
  }

  //////////////////////////////////
  //FoodOrdering QA methods - move//
  //////////////////////////////////
  @TRADEService
  @Action
  public void cancelAllActiveGoals() {
    synchronized (goalsLock) {
      for (Symbol agent: agentGoals.keySet()) {
        Goal g = agentGoals.get(agent);
        if (g != null && !g.getStatus().isTerminated()) {
          agentGoals.get(agent).cancel();
        }
      }
    }
  }

  @TRADEService
  @Action
  public void cancelAllPendingGoals() {
    synchronized (pendingGoalsLock) {
      while (!pendingGoals.isEmpty()) {
        cancelGoalInQueueIndex(pendingGoals.size()-1);
      }
    }
  }

  @TRADEService
  @Action
  public void cancelAllCurrentGoals() {
    synchronized (goalsLock) {
      synchronized (pendingGoalsLock) {
        cancelAllPendingGoals();
        cancelAllActiveGoals();
      }
    }
  }

  @TRADEService
  @Action
  public List<Predicate> getPendingGoalsPredicates() {
    List<Predicate> goalPreds = new ArrayList<>();
    synchronized (pendingGoalsLock) {
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pendingGoal = pendingGoalIterator.next();
        goalPreds.add(pendingGoal.getGoal().getPredicate());
      }
    }
    return goalPreds;
  }

  @TRADEService
  @Action
  public List<Predicate> getSystemGoalPredicates(Symbol actor) {
    return getSystemGoalPredicatesHelper(getUntypedSymbol(actor), new ArrayList<>());
  }

  private List<Predicate> getSystemGoalPredicatesHelper(Symbol actor, List<Predicate> goalPreds) {
    Goal g = agentGoals.get(actor);
    if (g != null && !g.getStatus().isTerminated()) {
      goalPreds.add(Factory.createPredicate(g.getStatus().toString(), g.getPredicate()));
    }

    AgentTeam agentTeam  = agentTeams.get(actor);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        goalPreds.addAll(getSystemGoalPredicates(member));
      }
    }

    return goalPreds;
  }

  @TRADEService
  @Action
  public Predicate getNextGoalPredicate() {
    synchronized (pendingGoalsLock) {
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      if (pendingGoalIterator.hasNext()) {
        return pendingGoalIterator.next().getGoal().getPredicate();
      } else {
        //TODO: what to return here?
        return Factory.createPredicate("none()");
      }
    }
  }

  @TRADEService
  @Action
  public Predicate getNextGoalPredicate(Symbol agent) {
    agent = getUntypedSymbol(agent);
    synchronized (pendingGoalsLock) {
      Iterator<PendingGoal> pendingGoalIterator = pendingGoals.descendingIterator();
      while (pendingGoalIterator.hasNext()) {
        PendingGoal pg = pendingGoalIterator.next();
        if (agent == getUntypedSymbol(pg.getGoal().getActor())) {
          return pg.getGoal().getPredicate();
        }
      }
    }
    //TODO: what to return here?
    return Factory.createPredicate("none()");
  }

  //Issue is this would include all dialogue goals and other goals which skipped the queue
  //Would need to track separately if we just wanted top level 'system' goals. They are technically still tracked here
  //  until a new active goal starts for each agent though
  //@TRADEService
  //@Action
  //public Predicate getLastGoal() {
  //  Goal lastGoal = null;
  //  for (Goal g: getPastGoals()) {
  //    if (lastGoal == null || g.getEndTime() > lastGoal.getEndTime()) {
  //      lastGoal = g;
  //    }
  //  }
  //  if (lastGoal == null) {
  //    //TODO: what to return here?
  //    return Factory.createPredicate("none()");
  //  } else {
  //    return lastGoal.getPredicate();
  //  }
  //}
}
