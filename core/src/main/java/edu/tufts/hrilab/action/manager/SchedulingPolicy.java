package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;

import java.util.HashSet;
import java.util.Set;

/** Existing scheduling choices, independent of interpreter and collection mechanics. */
enum SchedulingPolicy {
  CONCURRENT(false, false, false),
  SINGLE_GOAL(true, true, false),
  QUEUE(true, true, true);

  private final boolean requireAgent;
  private final boolean holdAgent;
  final boolean queueConflicts;

  SchedulingPolicy(boolean requireAgent, boolean holdAgent, boolean queueConflicts) {
    this.requireAgent = requireAgent;
    this.holdAgent = holdAgent;
    this.queueConflicts = queueConflicts;
  }

  static int comparePriority(PriorityTier firstTier, long firstPriority, long firstId,
                             PriorityTier secondTier, long secondPriority, long secondId) {
    int comparison = firstTier.compareTo(secondTier);
    if (comparison == 0) {
      comparison = Long.compare(firstPriority, secondPriority);
    }
    return comparison == 0 ? Long.compare(secondId, firstId) : comparison;
  }

  Set<Symbol> requiredResources(GoalInfo goal, Symbol agent) {
    if (goal.priorityTier == PriorityTier.SKIPPENDING) {
      return new HashSet<>();
    }
    Set<Symbol> resources = new HashSet<>();
    if (this == SINGLE_GOAL) {
      resources.addAll(heldResources(goal, agent));
    } else {
      if (!goal.isLearningGoal) {
        resources.add(Factory.createSymbol("learning"));
      }
      resources.add(Factory.createSymbol("freeze"));
    }
    if (requireAgent) {
      resources.add(agent);
    }
    return resources;
  }

  Set<Symbol> heldResources(GoalInfo goal, Symbol agent) {
    Set<Symbol> resources = new HashSet<>();
    if (goal.priorityTier == PriorityTier.SKIPPENDING) {
      return resources;
    }
    String action = goal.goal == null ? goal.unboundGoal.getName() : goal.goal.getName();
    if (!goal.isLearningGoal && (action.equals("learnAction") || action.equals("freeze"))) {
      resources.add(Factory.createSymbol(action.equals("learnAction") ? "learning" : "freeze"));
      if (this == QUEUE) {
        return resources;
      }
    }
    if (holdAgent) {
      resources.add(agent);
    }
    return resources;
  }
}
