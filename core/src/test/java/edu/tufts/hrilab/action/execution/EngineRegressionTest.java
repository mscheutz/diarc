package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static org.junit.Assert.*;

public class EngineRegressionTest {
  private final Symbol actor = Factory.createSymbol("self");
  private final RootContext root = new RootContext(new ActionConstraints(), null);

  public static class NormService {
    boolean allowed = true;
    boolean throwsException;
    int calls;

    @TRADEService
    public Justification getViolations(Symbol actor, StateMachine stateMachine) throws TRADEException {
      calls++;
      if (throwsException) {
        throw new IllegalStateException("Norm service failed");
      }
      return new ConditionJustification(allowed);
    }
  }

  public static class AgentService {
    @TRADEService
    public Set<String> getAllDiarcAgentsForActor(Symbol actor, Boolean includeGroups) {
      return Set.of("robot");
    }
  }

  private static class ActorContext extends Context {
    private final Symbol actor;

    ActorContext(Context parent, Symbol actor) {
      super(parent, null, "regression");
      this.actor = actor;
    }

    @Override
    public Object getArgumentValue(String name) {
      return "?actor".equals(name) ? actor : super.getArgumentValue(name);
    }

    @Override
    public Context copy(Context parent) {
      return new ActorContext(parent, actor);
    }
  }

  @Test(timeout = 2000)
  public void normVerificationCompletesAndPropagatesFailures() throws TRADEException {
    NormService service = new NormService();
    TRADE.registerAllServices(service, List.of());
    try {
      for (boolean allowed : new boolean[]{true, false}) {
        service.allowed = allowed;
        Context context = new ActorContext(root, actor);
        context.setStatus(ActionStatus.VERIFYING_EFFECTS);
        Justification result = new StepExecution(root, null).finishStep(context);
        assertEquals(allowed, result.getValue());
        assertEquals(allowed ? ActionStatus.SUCCESS : ActionStatus.FAIL_NORMS, context.getStatus());
      }
      assertEquals(2, service.calls);
    } finally {
      TRADE.deregister(service);
    }
  }

  @Test(timeout = 2000)
  public void normServiceErrorsFailExplicitly() throws TRADEException {
    NormService service = new NormService();
    service.throwsException = true;
    TRADE.registerAllServices(service, List.of());
    try {
      Context context = new ActorContext(root, actor);
      context.setStatus(ActionStatus.VERIFYING_EFFECTS);
      assertFalse(new StepExecution(root, null).finishStep(context).getValue());
      assertEquals(ActionStatus.FAIL_NORMS, context.getStatus());
      assertFalse(context.getJustification().getFailureReason().isEmpty());
    } finally {
      TRADE.deregister(service);
    }
  }

  @Test(timeout = 2000)
  public void completionWithoutNormServiceStillSucceeds() {
    Context context = new ActorContext(root, actor);
    context.setStatus(ActionStatus.VERIFYING_EFFECTS);
    assertTrue(new StepExecution(root, null).finishStep(context).getValue());
    assertEquals(ActionStatus.SUCCESS, context.getStatus());
  }

  @Test(timeout = 2000)
  public void interpreterExceptionTerminatesGoalAndClearsRunningFlag() throws ReflectiveOperationException {
    Context context = new ActorContext(root, actor) {
      @Override
      public void doStep() {
        throw new IllegalStateException("Regression execution failure");
      }
    };
    Goal goal = Goal.createGoal(actor, Factory.createPredicate("regression(self)"));
    ActionInterpreter interpreter = ActionInterpreter.createInterpreterFromContext(goal, context);
    assertEquals(ActionStatus.FAIL, interpreter.call());
    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertEquals(ActionStatus.FAIL, context.getStatus());
    assertFalse(goal.getFailConditions().getValue());
    Field running = ActionInterpreter.class.getDeclaredField("isRunning");
    running.setAccessible(true);
    assertFalse(running.getBoolean(interpreter));
  }

  @Test(timeout = 2000)
  public void exceptionDuringStartNotificationAlsoTerminatesGoal() {
    Goal goal = Goal.createGoal(actor, Factory.createPredicate("startFailure(self)"));
    goal.addListener(info -> {
      if (info.status == GoalStatus.ACTIVE) {
        throw new IllegalStateException("Start notification failed");
      }
    });
    Context context = new ActorContext(root, actor);
    ActionInterpreter interpreter = ActionInterpreter.createInterpreterFromContext(goal, context);
    assertEquals(ActionStatus.FAIL, interpreter.call());
    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertEquals(ActionStatus.FAIL, context.getStatus());
  }

  private Context executeControls(String... commands) {
    List<EventSpec> events = new ArrayList<>();
    for (String command : commands) {
      events.add(new EventSpec(EventSpec.EventType.CONTROL, command));
    }
    Context context = root.setupNextStep(new ScriptParser(events));
    Goal goal = Goal.createGoal(actor, Factory.createPredicate("controls(self)"));
    assertEquals(ActionStatus.SUCCESS, ActionInterpreter.createInterpreterFromContext(goal, context).call());
    assertEquals(GoalStatus.SUCCEEDED, goal.getStatus());
    return context;
  }

  @Test(timeout = 2000)
  public void falseAndCompoundConditionsExecuteWithoutSyntaxFailures() {
    assertFalse(executeControls("false").getLogicalValue());
    assertTrue(executeControls("not", "false", "endnot").getLogicalValue());
    assertFalse(executeControls("and", "true", "false", "endand").getLogicalValue());
    assertTrue(executeControls("or", "false", "true", "endor").getLogicalValue());
  }

  @Test(timeout = 2000)
  public void falseRemainsFalseAfterCopyAndReset() {
    Context context = executeControls("false");
    Context copy = context.copy(root);
    assertFalse(copy.getLogicalValue());
    copy.resetContext();
    Goal goal = Goal.createGoal(actor, Factory.createPredicate("copiedFalse(self)"));
    assertEquals(ActionStatus.SUCCESS, ActionInterpreter.createInterpreterFromContext(goal, copy).call());
    assertFalse(copy.getLogicalValue());
  }

  @Test(timeout = 2000)
  public void whileFalseSkipsItsBodyAndSurvivesReset() {
    List<ActionDBEntry> actions = new ActionScriptLanguageParser().parseFromString("regression", null,
            "() = neverRuns() { while (false) { act:missingAction(); } }");
    assertNotNull(actions);
    Context loop = root.setupNextStep(new ScriptParser(actions.get(0).getEventSpecs()));
    for (int i = 0; i < 2; i++) {
      Goal goal = Goal.createGoal(actor, Factory.createPredicate("neverRuns(self)"));
      assertEquals(ActionStatus.SUCCESS, ActionInterpreter.createInterpreterFromContext(goal, loop).call());
      assertEquals(1, loop.getChildContexts().size());
      assertFalse(loop.getChildContexts().get(0).getLogicalValue());
      loop.resetContext();
    }
  }

  @Test(timeout = 2000)
  public void observerActorSearchAscendsAndTerminatesWithoutAnAgent() throws TRADEException {
    AgentService service = new AgentService();
    TRADE.registerAllServices(service, List.of());
    try {
      Symbol robot = Factory.createSymbol("robot");
      Context grandparent = new ActorContext(root, robot);
      Context parent = new ActorContext(grandparent, Factory.createSymbol("human"));
      ActionDBEntry action = new ActionDBEntry.Builder("observerRegression").build(false);
      ActionContext context = new ActionContext(parent, null, action, List.of(), List.of(), actor);
      assertEquals(robot, context.getDiarcActor());
      ActionContext ownActor = new ActionContext(parent, null, action, List.of(), List.of(), robot);
      assertEquals(robot, ownActor.getDiarcActor());
      ActionContext noRobot = new ActionContext(new ActorContext(root, actor), null,
              action, List.of(), List.of(), actor);
      assertEquals(actor, noRobot.getDiarcActor());
    } finally {
      Database.destroyInstance();
      TRADE.deregister(service);
    }
  }
}
