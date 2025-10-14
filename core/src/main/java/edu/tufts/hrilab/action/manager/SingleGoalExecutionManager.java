package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Collection;
import java.util.Set;

public class SingleGoalExecutionManager extends ExecutionManager {
  private static final Logger log = LoggerFactory.getLogger(SingleGoalExecutionManager.class);

  public SingleGoalExecutionManager() {
    super();
  }

  protected void init(StateMachine sm, RootContext rootContext, Collection<String> groups) {
    super.init(sm, rootContext,groups);
  }

  /**
   * Returns the set of all Resources required to be available during execution of the supplied goal.
   */
  @Override
  protected Set<Resource> getRequiredResourcesForGoal(Goal goal) {
    if (goal.getHeldResources() != null) {
      return goal.getHeldResources();
    } else {
      Set<Resource> resourceSet = super.getHeldResourcesForGoal(goal);
      if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
        for (Symbol agent : getDescendants(goal.getActor())) {
          resourceSet.add(agentTeams.get(agent).getResource(agent));
        }
      }
      goal.setRequiredResources(resourceSet);

      return resourceSet;
    }
  }

  /**
   * Returns the set of all Resources that will be held by the supplied goal during its execution.
   */
  @Override
  protected Set<Resource> getHeldResourcesForGoal(Goal goal) {
    if (goal.getHeldResources() != null) {
      return goal.getHeldResources();
    } else {
      Set<Resource> resourceSet = super.getHeldResourcesForGoal(goal);
      if (goal.getPriorityTier() != PriorityTier.SKIPPENDING) {
        for (Symbol agent : getDescendants(goal.getActor())) {
          resourceSet.add(agentTeams.get(agent).getResource(agent));
        }
        goal.setHeldResources(resourceSet);
      }

      return resourceSet;
    }
  }

  //TODO: override method (or change in base class) to get more descriptive failures indicating what the current conflicting task is
}
