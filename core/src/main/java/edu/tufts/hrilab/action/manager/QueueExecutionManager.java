/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
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

  protected void init(StateMachine sm, RootContext rootContext, Collection<String> groups) {
    super.init(sm, rootContext,groups);
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

    //Assign as many pending goals as possible (in order of priority) with resources freed up by this active goal
    // completing
    if (status.isTerminated() && !pendingGoals.isEmpty() && consumedResources(g)) {
      activateValidPendingGoals();
    }
  }

  @Override
  protected void handleConflictingLowerPriorityGoal(Goal g) {
    //QueueEM: Simply leave in the pending queue rather than rejecting
  }

  //TODO: extend definition of resources past just whole agents/teams
  @Override
  protected Set<Resource> getRequiredResourcesForGoal(Goal goal) {
    if (goal.getRequiredResources() != null) {
      return goal.getRequiredResources();
    } else {
      Set<Resource> resourceSet = new HashSet<>();

      //Iterate over all gathered agents and pool agent wide resources
      for (Symbol agent : getDescendants(goal.getActor())) {
        //If the goal skips the queue, it has no resource dependencies.
        if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
          //If it is a learning goal, only requires the agent-wide lock to be available.
          if (goal.isLearningGoal()) {
            resourceSet.add(agentTeams.get(agent).getResource(agent));
          }
          //Otherwise it requires both the learning and agent-wide lock to be available
          else {
            resourceSet.addAll(agentTeams.get(agent).getResources());
          }
        }
      }
      //Store required resources in goal to prevent constantly recomputing set
      goal.setRequiredResources(resourceSet);

      return resourceSet;
    }
  }
  @Override
  protected Set<Resource> getHeldResourcesForGoal(Goal goal) {
    if (goal.getHeldResources() != null) {
      return goal.getHeldResources();
    } else {
      Set<Resource> resourceSet = new HashSet<>();

      //Iterate over all gathered agents and pool agent wide resources
      for (Symbol agent : getDescendants(goal.getActor())) {
        //If the goal skips the queue, it holds no resources
        if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
          //Exception to the rule - action learning itself takes only the learning lock
          if (goal.getPredicate().getName().equals("learnAction")) {
            resourceSet.add(agentTeams.get(agent).getResource(Factory.createSymbol("learning")));
          }
          //If it is a learning goal, takes just the agent-wide lock.
          if (goal.isLearningGoal()) {
            resourceSet.add(agentTeams.get(agent).getResource(agent));
          }
          //Otherwise it takes both the learning and agent-wide lock
          else {
            resourceSet.addAll(agentTeams.get(agent).getResources());
          }
        }
      }
      //Store required resources in goal to prevent constantly recomputing set
      goal.setHeldResources(resourceSet);

      return resourceSet;
    }
  }

  @TRADEService
  @Action
  public void cancelAllSystemGoals() {
    log.info("[cancelAllSystemGoals] in method");
    synchronized (goalsLock) {
      Set<Goal> goalsToCancel = new HashSet<>();
      log.trace("[cancelAllSystemGoals] have goalsLock");
      for (Symbol agent: agentGoals.keySet()) {
        Goal g = agentGoals.get(agent);
        if (g != null && !g.getStatus().isTerminated()) {
          goalsToCancel.add(g);
        }
      }
      for (Goal g: goalsToCancel) {
        cancelGoal(g.getId());
      }
    }
    log.trace("[cancelAllSystemGoals] released goalsLock");
  }

  //TODO: Can we move these methods to the base EM or use those which exist in core.asl?
  //      What does the 'current goal' mean in the general sense? If there are
  //      N active goals, do we pick the oldest, most recent, one at random?

  @Action
  @TRADEService
  public List<Long> cancelSystemGoals(Symbol actor) {
    log.debug("[cancelSystemGoals] in method");
    List<Long> canceledIds = new ArrayList<>();
    actor = getUntypedSymbol(actor);
    long goalId = getSystemGid(actor);
    if (goalId != -1) {
      if (cancelGoal(goalId)) {
        canceledIds.add(goalId);
      }
    }

    AgentTeam agentTeam = agentTeams.get(actor);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        canceledIds.addAll(cancelSystemGoals(member));
      }
    }

    return canceledIds;
  }

  @Action
  @TRADEService
  public List<Long> suspendSystemGoals(Symbol actor) {
    log.info("[suspendSystemGoals] {}", actor);
    List<Long> suspendedIds = new ArrayList<>();
    actor = getUntypedSymbol(actor);
    long goalId = getSystemGid(actor);
    if (goalId != -1) {
      if (suspendGoal(goalId)) {
        suspendedIds.add(goalId);
      }
    }

    AgentTeam agentTeam = agentTeams.get(actor);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        suspendedIds.addAll(suspendSystemGoals(member));
      }
    }

    log.info("[suspendSystemGoals] done {}", actor);
    return suspendedIds;
  }

  @Action
  @TRADEService
  public List<Long> resumeSystemGoals(Symbol actor) {
    log.debug("[resumeSystemGoals] in method");
    List<Long> resumedIds = new ArrayList<>();
    actor = getUntypedSymbol(actor);
    long goalId = getSystemGid(actor);
    if (goalId != -1) {
      if (resumeGoal(goalId)) {
        resumedIds.add(goalId);
      }
    }

    AgentTeam agentTeam = agentTeams.get(actor);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        resumedIds.addAll(resumeSystemGoals(member));
      }
    }

    return resumedIds;
  }

  private Long getSystemGid(Symbol actor) {
    if (agentGoals.get(actor) != null) {
      return agentGoals.get(actor).getId();
    } else {
      return -1L;
    }
  }

  @Override
  public List<Predicate> getSystemGoalsPredicates(Symbol actor) {
    log.trace("[getSystemGoalsPredicates] {}", actor);
    return getSystemGoalsPredicatesHelper(getUntypedSymbol(actor), new ArrayList<>());
  }

  private List<Predicate> getSystemGoalsPredicatesHelper(Symbol actor, List<Predicate> goalPreds) {
    Goal g = agentGoals.get(actor);
    if (g != null && !g.getStatus().isTerminated()) {
      goalPreds.add(Factory.createPredicate(g.getStatus().toString(), g.getPredicate()));
    }

    AgentTeam agentTeam  = agentTeams.get(actor);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        goalPreds.addAll(getSystemGoalsPredicates(member));
      }
    }

    return goalPreds;
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
