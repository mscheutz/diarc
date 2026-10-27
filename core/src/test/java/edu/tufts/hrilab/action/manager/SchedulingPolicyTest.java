package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.Assert.*;

public class SchedulingPolicyTest {
  private static final Symbol SELF = Factory.createSymbol("self");

  private ExecutionManager initialize(ExecutionManager manager) {
    manager.rootAgent = SELF;
    manager.agentHierarchy = Map.of(SELF, Set.of(SELF));
    manager.agentTeams.put(SELF, new AgentTeam(SELF, manager));
    return manager;
  }

  private Set<String> names(Set<Resource> resources) {
    return resources.stream().map(resource -> resource.getName().getName()).collect(Collectors.toSet());
  }

  @Test
  public void allCompatibilityManagersPreserveTheirLearningFreezeAndSkipPendingClaims() {
    for (ExecutionManager manager : List.of(initialize(new ExecutionManager()),
            initialize(new SingleGoalExecutionManager()), initialize(new QueueExecutionManager()))) {
      for (String action : List.of("test", "learnAction", "freeze")) {
        for (boolean teaching : List.of(false, true)) {
          for (PriorityTier tier : List.of(PriorityTier.NORMAL, PriorityTier.SKIPPENDING)) {
            Goal goal = Goal.createGoal(Factory.createPredicate(action + "(self)"));
            goal.setIsLearningGoal(teaching);
            goal.setPriorityTier(tier);
            Set<String> required = new HashSet<>();
            Set<String> held = new HashSet<>();
            if (tier != PriorityTier.SKIPPENDING) {
              if (!teaching && !action.equals("test")) {
                held.add(action.equals("learnAction") ? "learning" : "freeze");
              }
              if (manager instanceof SingleGoalExecutionManager) {
                held.add("self");
                required.addAll(held);
              } else {
                required.add("freeze");
                if (!teaching) {
                  required.add("learning");
                }
                if (manager instanceof QueueExecutionManager) {
                  required.add("self");
                  if (held.isEmpty()) {
                    held.add("self");
                  }
                }
              }
            }
            assertEquals(manager.getClass() + " required " + action + teaching + tier,
                    required, names(manager.getRequiredResourcesForGoal(goal)));
            assertEquals(manager.getClass() + " held " + action + teaching + tier,
                    held, names(manager.getHeldResourcesForGoal(goal)));
          }
        }
      }
    }
  }

  @Test
  public void singleGoalRequiredClaimsCopyExplicitHeldClaimsWithoutAliasing() {
    ExecutionManager manager = initialize(new SingleGoalExecutionManager());
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    Resource custom = new Resource(null, Factory.createSymbol("custom"));
    Set<Resource> held = new HashSet<>(Set.of(custom));
    goal.setHeldResources(held);
    Set<Resource> required = manager.getRequiredResourcesForGoal(goal);
    assertEquals(held, required);
    assertNotSame(held, required);
    held.clear();
    assertEquals(Set.of(custom), required);
  }

  @Test
  public void concurrentRejectingAndQueuingDispositionsRemainDistinct() {
    ExecutionManager concurrent = initialize(new ExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        return goal.setAsStarted();
      }
    });
    ExecutionManager single = initialize(new SingleGoalExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        return goal.setAsStarted();
      }
    });
    ExecutionManager queued = initialize(new QueueExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        return goal.setAsStarted();
      }
    });
    for (ExecutionManager manager : List.of(concurrent, single, queued)) {
      Goal first = Goal.createGoal(Factory.createPredicate("test(self)"));
      Goal second = Goal.createGoal(Factory.createPredicate("test(self)"));
      manager.submitGoal(first);
      manager.submitGoal(second);
      assertEquals(GoalStatus.ACTIVE, first.getStatus());
      assertEquals(manager == concurrent ? GoalStatus.ACTIVE
              : manager == single ? GoalStatus.FAILED : GoalStatus.PENDING, second.getStatus());
    }
  }

  @Test
  public void orderingPreservesTiersExtremePrioritiesAndOlderIdTieBreaks() {
    assertTrue(SchedulingPolicy.comparePriority(PriorityTier.HIGH, Long.MIN_VALUE, 10,
            PriorityTier.LOW, Long.MAX_VALUE, 1) > 0);
    assertTrue(SchedulingPolicy.comparePriority(PriorityTier.NORMAL, Long.MAX_VALUE, 10,
            PriorityTier.NORMAL, Long.MIN_VALUE, 1) > 0);
    assertTrue(SchedulingPolicy.comparePriority(PriorityTier.NORMAL, 1, 1,
            PriorityTier.NORMAL, 1, 10) > 0);
    assertEquals(0, SchedulingPolicy.comparePriority(PriorityTier.NORMAL, 1, 1,
            PriorityTier.NORMAL, 1, 1));
  }
}
