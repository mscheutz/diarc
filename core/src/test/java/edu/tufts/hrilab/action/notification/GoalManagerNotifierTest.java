package edu.tufts.hrilab.action.notification;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import static org.junit.Assert.*;

public class GoalManagerNotifierTest {
  public static class Callbacks {
    Consumer<GoalInfo> primary = info -> {};
    Consumer<GoalInfo> secondary = info -> {};
    Consumer<GoalInfo> tertiary = info -> {};

    @TRADEService
    public void stage2Primary(GoalInfo info) {
      primary.accept(info);
    }

    @TRADEService
    public void stage2Secondary(GoalInfo info) {
      secondary.accept(info);
    }

    @TRADEService
    public void stage2Tertiary(GoalInfo info) {
      tertiary.accept(info);
    }
  }

  private TRADEServiceInfo service(String name) throws Exception {
    return TRADE.getAvailableService(new TRADEServiceConstraints().name(name).argTypes(GoalInfo.class));
  }

  private GoalInfo info() {
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    goal.setAsStarted();
    return goal.getInfo();
  }

  @Test
  public void callbacksReceiveIndependentVersionedSnapshots() throws Exception {
    Callbacks callbacks = new Callbacks();
    GoalManagerNotifier notifier = new GoalManagerNotifier();
    List<GoalInfo> received = new ArrayList<>();
    callbacks.primary = info -> {
      info.status = GoalStatus.FAILED;
      info.statusVersion = 999;
    };
    callbacks.secondary = received::add;
    TRADE.registerAllServices(callbacks, List.of());
    try {
      notifier.registerNotification(service("stage2Primary"), NotificationType.GOAL_STATUS_CHANGE);
      notifier.registerNotification(service("stage2Secondary"), NotificationType.GOAL_STATUS_CHANGE);
      GoalInfo original = info();
      notifier.goalStatusChanged(original);
      assertEquals(GoalStatus.ACTIVE, original.status);
      assertEquals(1, original.statusVersion);
      assertEquals(GoalStatus.ACTIVE, received.get(0).status);
      assertEquals(1, received.get(0).statusVersion);
    } finally {
      TRADE.deregister(callbacks);
    }
  }

  @Test(timeout = 3000)
  public void callbacksCanWaitForRegistrationOnAnotherThreadWithoutHoldingRegistryLock() throws Exception {
    Callbacks callbacks = new Callbacks();
    GoalManagerNotifier notifier = new GoalManagerNotifier();
    ExecutorService executor = Executors.newSingleThreadExecutor();
    AtomicBoolean registered = new AtomicBoolean();
    TRADE.registerAllServices(callbacks, List.of());
    try {
      TRADEServiceInfo tertiary = service("stage2Tertiary");
      callbacks.primary = info -> {
        try {
          executor.submit(() -> notifier.registerNotification(tertiary, NotificationType.GOAL_STATUS_CHANGE))
                  .get(1, TimeUnit.SECONDS);
          registered.set(true);
        } catch (Exception e) {
          throw new AssertionError(e);
        }
      };
      notifier.registerNotification(service("stage2Primary"), NotificationType.GOAL_STATUS_CHANGE);
      notifier.goalStatusChanged(info());
      assertTrue(registered.get());
    } finally {
      executor.shutdownNow();
      TRADE.deregister(callbacks);
    }
  }

  @Test
  public void registrationChangesAffectNextDeliveryNotCurrentSnapshot() throws Exception {
    Callbacks callbacks = new Callbacks();
    GoalManagerNotifier notifier = new GoalManagerNotifier();
    List<String> delivered = new ArrayList<>();
    TRADE.registerAllServices(callbacks, List.of());
    try {
      TRADEServiceInfo primary = service("stage2Primary");
      TRADEServiceInfo secondary = service("stage2Secondary");
      TRADEServiceInfo tertiary = service("stage2Tertiary");
      callbacks.primary = info -> {
        delivered.add("primary");
        notifier.unregisterNotification(primary, NotificationType.GOAL_STATUS_CHANGE);
        notifier.unregisterNotification(secondary, NotificationType.GOAL_STATUS_CHANGE);
        notifier.registerNotification(tertiary, NotificationType.GOAL_STATUS_CHANGE);
      };
      callbacks.secondary = info -> delivered.add("secondary");
      callbacks.tertiary = info -> delivered.add("tertiary");
      notifier.registerNotification(primary, NotificationType.GOAL_STATUS_CHANGE);
      notifier.registerNotification(secondary, NotificationType.GOAL_STATUS_CHANGE);
      notifier.goalStatusChanged(info());
      assertEquals(2, delivered.size());
      assertTrue(delivered.containsAll(List.of("primary", "secondary")));
      notifier.goalStatusChanged(info());
      assertEquals(3, delivered.size());
      assertEquals("tertiary", delivered.get(2));
    } finally {
      TRADE.deregister(callbacks);
    }
  }

  @Test
  public void oneFailingCallbackDoesNotSuppressOtherCallbacks() throws Exception {
    Callbacks callbacks = new Callbacks();
    GoalManagerNotifier notifier = new GoalManagerNotifier();
    List<GoalInfo> delivered = new ArrayList<>();
    callbacks.primary = info -> {
      throw new IllegalStateException("Callback failed");
    };
    callbacks.secondary = delivered::add;
    TRADE.registerAllServices(callbacks, List.of());
    try {
      notifier.registerNotification(service("stage2Primary"), NotificationType.GOAL_STATUS_CHANGE);
      notifier.registerNotification(service("stage2Secondary"), NotificationType.GOAL_STATUS_CHANGE);
      notifier.goalStatusChanged(info());
      assertEquals(1, delivered.size());
    } finally {
      TRADE.deregister(callbacks);
    }
  }
}
