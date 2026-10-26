package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalSchedulingExplanation;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.GoalWaitResult;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.List;

import static org.junit.Assert.*;

public class StageTwoServicesTest {
  @Test
  public void versionedWaitAndExplanationAreAvailableThroughTrade() throws Exception {
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    ExecutionManager manager = new ExecutionManager() {
      @Override
      public Goal getGoal(long gid) {
        return gid == goal.getId() ? goal : null;
      }
    };
    goal.addListener(manager);
    GoalManagerComponent component = new GoalManagerComponent();
    Field field = GoalManagerComponent.class.getDeclaredField("em");
    field.setAccessible(true);
    field.set(component, manager);
    TRADE.registerAllServices(component, List.of());
    try {
      GoalInfo before = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalInfo")
              .argTypes(Long.class)).call(GoalInfo.class, goal.getId());
      goal.setAsStarted();
      GoalWaitResult update = TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForGoalUpdate")
              .argTypes(Long.class, Long.class, Long.class)).call(
              GoalWaitResult.class, goal.getId(), before.statusVersion, 10L);
      assertEquals(GoalWaitResult.Outcome.UPDATED, update.outcome);
      assertEquals(GoalStatus.ACTIVE, update.getGoalInfo().status);
      GoalSchedulingExplanation explanation = TRADE.getAvailableService(
              new TRADEServiceConstraints().name("explainGoalScheduling").argTypes(Long.class))
              .call(GoalSchedulingExplanation.class, goal.getId());
      assertEquals(goal.getId(), explanation.getGoalInfo().gid);
      assertEquals(1, explanation.getGoalInfo().statusVersion);
      assertEquals(GoalSchedulingExplanation.Location.UNKNOWN,
              component.explainGoalScheduling(Long.MAX_VALUE).location);
      assertNull(component.getGoalInfo(Long.MAX_VALUE));
    } finally {
      TRADE.deregister(component);
    }
  }
}
