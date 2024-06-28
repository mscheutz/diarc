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
    if (goal.getRequiredResources() != null) {
      return goal.getRequiredResources();
    } else {
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
      //Store required resources in goal to prevent constantly recomputing set
      goal.setRequiredResources(resourceSet);

      return resourceSet;
    }
  }

  //TODO: replace all usages of all below methods (from current to system)
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
  public long cancelSystemGoal() {
    return cancelSystemGoal(rootAgent);
  }

  //TODO: return value doesn't make sense anymore if acting on all children
  @Action
  @TRADEService
  public long cancelSystemGoal(Symbol agent) {
    log.debug("[cancelSystemGoal] in method");
    agent = getUntypedSymbol(agent);
    long goalId = getSystemGid(agent);
    if (goalId != -1) {
      cancelGoal(goalId);
    }

    AgentTeam agentTeam = agentTeams.get(agent);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        cancelSystemGoal(member);
      }
    }

    return -1;
  }

  @Action
  @TRADEService
  public long suspendSystemGoal() {
    return suspendSystemGoal(rootAgent);
  }

  //TODO: return value doesn't make sense anymore if acting on all children
  @Action
  @TRADEService
  public long suspendSystemGoal(Symbol agent) {
    log.debug("[suspendSystemGoal] in method");
    agent = getUntypedSymbol(agent);
    long goalId = getSystemGid(agent);
    if (goalId != -1) {
      suspendGoal(goalId);
    }

    AgentTeam agentTeam = agentTeams.get(agent);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        suspendSystemGoal(member);
      }
    }

    return -1;
  }

  @Action
  @TRADEService
  public long resumeSystemGoal() {
    return resumeSystemGoal(rootAgent);
  }

  //TODO: return value doesn't make sense anymore if acting on all children
  @Action
  @TRADEService
  public long resumeSystemGoal(Symbol agent) {
    log.debug("[resumeSystemGoal] in method");
    agent = getUntypedSymbol(agent);
    long goalId = getSystemGid(agent);
    if (goalId != -1) {
      resumeGoal(goalId);
    }

    AgentTeam agentTeam = agentTeams.get(agent);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        resumeSystemGoal(member);
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

  @Override
  public List<Predicate> getActiveGoalsPredicates(Symbol actor) {
    log.trace("[getActiveGoalsPredicates] {}", actor);
    return getActiveGoalsPredicatesHelper(getUntypedSymbol(actor), new ArrayList<>());
  }

  private List<Predicate> getActiveGoalsPredicatesHelper(Symbol actor, List<Predicate> goalPreds) {
    Goal g = agentGoals.get(actor);
    if (g != null && !g.getStatus().isTerminated()) {
      goalPreds.add(Factory.createPredicate(g.getStatus().toString(), g.getPredicate()));
    }

    AgentTeam agentTeam  = agentTeams.get(actor);
    if (agentTeam != null) {
      for (Symbol member: agentTeam.getMemberNames()) {
        goalPreds.addAll(getActiveGoalsPredicates(member));
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
