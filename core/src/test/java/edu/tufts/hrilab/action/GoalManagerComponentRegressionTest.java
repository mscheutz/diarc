/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalFilter;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.fol.Factory;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.Options;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.List;

import static org.junit.Assert.*;

public class GoalManagerComponentRegressionTest {
  private static GoalManagerComponent parse(String... args) throws Exception {
    GoalManagerComponent manager = new GoalManagerComponent();
    Options options = new Options();
    manager.additionalUsageInfo().forEach(options::addOption);
    manager.parseArgs(new DefaultParser().parse(options, args));
    return manager;
  }

  private static Object field(Object owner, Class<?> type, String name) throws Exception {
    Field field = type.getDeclaredField(name);
    field.setAccessible(true);
    return field.get(owner);
  }

  @Test
  public void badStateWorksWithoutBadAction() throws Exception {
    Object constraints = field(parse("-badstate", "unsafe(self)"), GoalManagerComponent.class, "initialConstraints");
    assertEquals(List.of(Factory.createPredicate("unsafe(self)")),
        field(constraints, ActionConstraints.class, "forbiddenStates"));
    assertEquals(List.of(), field(constraints, ActionConstraints.class, "forbiddenActions"));
  }

  @Test
  public void badStateAndBadActionRemainIndependent() throws Exception {
    Object constraints = field(parse("-badaction", "dangerousAction", "-badstate", "unsafe(self)"),
        GoalManagerComponent.class, "initialConstraints");
    assertEquals(List.of(Factory.createPredicate("unsafe(self)")),
        field(constraints, ActionConstraints.class, "forbiddenStates"));
    assertEquals(List.of("dangerousAction"), field(constraints, ActionConstraints.class, "forbiddenActions"));
  }

  @Test
  public void declaredPerformanceDirectorySpellingOverridesTheDefault() throws Exception {
    assertEquals("custom/performance", field(parse("-performancefileDir", "custom/performance"),
        GoalManagerComponent.class, "performanceFileDir"));
  }

  private static GoalManagerComponent withNextGoal(Goal goal) throws Exception {
    GoalManagerComponent manager = new GoalManagerComponent();
    Field execution = GoalManagerComponent.class.getDeclaredField("em");
    execution.setAccessible(true);
    execution.set(manager, new ExecutionManager() {
      @Override
      public Goal getNextGoal() {
        return goal;
      }
    });
    return manager;
  }

  @Test
  public void everyNextQueryReturnsAnEmptyListWhenNoGoalIsPending() throws Exception {
    GoalManagerComponent manager = withNextGoal(null);
    var actor = Factory.createSymbol("self:agent");
    var predicate = Factory.createPredicate("task(self:agent)");
    assertEquals(List.of(), manager.getGoalPredicates(GoalFilter.NEXT));
    assertEquals(List.of(), manager.getGoalPredicatesForActor(GoalFilter.NEXT, actor));
    assertEquals(List.of(), manager.getGoalPredicates(GoalFilter.NEXT, predicate));
    assertEquals(List.of(), manager.getGoalInfos(GoalFilter.NEXT));
    assertEquals(List.of(), manager.getGoalInfosForActor(GoalFilter.NEXT, actor));
    assertEquals(List.of(), manager.getGoalInfos(GoalFilter.NEXT, predicate));
    assertEquals(List.of(), manager.getGoalIds(GoalFilter.NEXT));
    assertEquals(List.of(), manager.getGoalIdsForActor(GoalFilter.NEXT, actor));
    assertEquals(List.of(), manager.getGoalIds(GoalFilter.NEXT, predicate));
  }

  @Test
  public void nextQueriesStillReturnTheSelectedGoalAndRespectTheirFilters() throws Exception {
    Goal goal = Goal.createGoal(Factory.createPredicate("task(self:agent)"));
    GoalManagerComponent manager = withNextGoal(goal);
    var actor = Factory.createSymbol("self");
    assertEquals(List.of(goal.getGoalPredicate()), manager.getGoalPredicates(GoalFilter.NEXT));
    assertEquals(List.of(goal.getGoalPredicate()), manager.getGoalPredicatesForActor(GoalFilter.NEXT, actor));
    assertEquals(List.of(goal.getGoalPredicate()), manager.getGoalPredicates(GoalFilter.NEXT, goal.getGoalPredicate()));
    assertEquals(List.of(goal.getId()), manager.getGoalInfos(GoalFilter.NEXT).stream().map(info -> info.gid).toList());
    assertEquals(List.of(goal.getId()), manager.getGoalInfosForActor(GoalFilter.NEXT, actor)
        .stream().map(info -> info.gid).toList());
    assertEquals(List.of(goal.getId()), manager.getGoalInfos(GoalFilter.NEXT, goal.getGoalPredicate())
        .stream().map(info -> info.gid).toList());
    assertEquals(List.of(goal.getId()), manager.getGoalIds(GoalFilter.NEXT));
    assertEquals(List.of(goal.getId()), manager.getGoalIdsForActor(GoalFilter.NEXT, actor));
    assertEquals(List.of(goal.getId()), manager.getGoalIds(GoalFilter.NEXT, goal.getGoalPredicate()));
    assertTrue(manager.getGoalIdsForActor(GoalFilter.NEXT, Factory.createSymbol("other")).isEmpty());
    assertTrue(manager.getGoalIds(GoalFilter.NEXT, Factory.createPredicate("other(self)")).isEmpty());
  }
}
