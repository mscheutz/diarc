/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.asl;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.ExecutionDiagnostic;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import org.junit.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import static org.junit.Assert.*;

/** End-to-end tests through the native parser, Goal Manager, and Action Interpreter. */
public class NativeAslEnhancementsTest {
  private static GoalManagerComponent manager;
  private static final AtomicInteger names = new AtomicInteger();
  private final List<Long> goals = new ArrayList<>();
  private final List<ActionDBEntry> actions = new ArrayList<>();
  private static final String IMPORTS = """
      import edu.tufts.hrilab.action.execution.AslTask;
      import edu.tufts.hrilab.action.ActionStatus;
      import edu.tufts.hrilab.action.asl.CompiledAction;
      import java.lang.String;
      """;

  @BeforeClass public static void initialize() {
    manager = DiarcComponent.createInstance(GoalManagerComponent.class, true, false,
        "-beliefinitfile agents/agents.pl -nomemorymanager -asl native-asl-features.asl");
  }

  @AfterClass public static void shutdown() { manager.shutdownComponent(); }

  @After public void cleanup() {
    for (long goal : goals) {
      if (!manager.getGoalStatus(goal).isTerminated()) manager.cancelGoal(goal);
      assertTrue(manager.joinOnGoal(goal, 5000).isTerminated());
    }
    actions.forEach(Database.getInstance()::removeActionDBEntry);
  }

  private long run(String source) {
    String name = "aslNativeEnhancement" + names.incrementAndGet();
    List<ActionDBEntry> parsed = new ActionScriptLanguageParser()
        .parseStrict("native-enhancement-test.asl", null, IMPORTS + source.replace("ROOT", name));
    actions.addAll(parsed);
    parsed.forEach(Database.getInstance()::addActionDBEntry);
    long goal = manager.submitGoal(Factory.createPredicate(name + "(self:agent)"));
    goals.add(goal);
    return goal;
  }

  private long run(ActionDBEntry action) {
    actions.add(action);
    Database.getInstance().addActionDBEntry(action);
    long goal = manager.submitGoal(Factory.createPredicate(action.getName() + "(self:agent)"));
    goals.add(goal);
    return goal;
  }

  private void succeeds(String source) {
    long goal = run(source);
    assertEquals(String.valueOf(manager.getGoalExecutionDiagnostic(goal)),
        GoalStatus.SUCCEEDED, manager.joinOnGoal(goal, 5000));
  }

  private ExecutionDiagnostic fails(String source) {
    long goal = run(source);
    assertEquals(GoalStatus.FAILED, manager.joinOnGoal(goal, 5000));
    ExecutionDiagnostic diagnostic = manager.getGoalExecutionDiagnostic(goal);
    assertNotNull(diagnostic);
    return diagnostic;
  }

  private static String quote(String source) {
    return "\"" + source.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", " ") + "\"";
  }

  @Test(timeout = 15000)
  public void typedExpressionsSupportPrecedenceBooleanStringsObjectsAndLoops() {
    succeeds("""
        () = ROOT() {
          long !n = 2 + 3 * 4;
          java.lang.Boolean !flag;
          String !text;
          edu.tufts.hrilab.fol.Symbol !place;
          edu.tufts.hrilab.fol.Symbol !copy;
          !flag = !n == 14 && true;
          !text = "value=" + !n;
          !place = lab:location;
          !copy = !place;
          while (!n gt 10) { !n = !n-1; }
          if (!flag && !n == 10 && !text == "value=14" && !copy == !place) {
          } else { exit(FAIL); }
        }
        """);
  }

  @Test(timeout = 20000)
  public void standaloneBlocksInsideForSerializeAndExecuteWithTheSameScope() {
    String originalName = "aslNestedBlockLoop" + names.incrementAndGet();
    String restoredName = originalName + "Reloaded";
    String source = """
        () = ROOT() {
          int !iterations = 0;
          for (!i=0; !i lt 3; !i++) {
            block { int !temporary = !i; }
            !iterations = !iterations + 1;
          }
          if (!iterations == 3) { } else { exit(FAIL); }
        }
        """.replace("ROOT", originalName);
    ActionDBEntry original = new ActionScriptLanguageParser()
        .parseStrict("nested-block-runtime.asl", null, IMPORTS + source).get(0);
    String written = new ActionScriptLanguageWriter().writeAction(original);
    ActionDBEntry restored = new ActionScriptLanguageParser()
        .parseStrict("nested-block-runtime.asl", null, written.replace(originalName, restoredName)).get(0);

    long originalGoal = run(original);
    long restoredGoal = run(restored);
    assertEquals(String.valueOf(manager.getGoalExecutionDiagnostic(originalGoal)),
        GoalStatus.SUCCEEDED, manager.joinOnGoal(originalGoal, 5000));
    assertEquals(String.valueOf(manager.getGoalExecutionDiagnostic(restoredGoal)),
        GoalStatus.SUCCEEDED, manager.joinOnGoal(restoredGoal, 5000));
  }

  @Test(timeout = 15000)
  public void taskResultsArePrivateUntilExplicitJoinAndIncludeDirectActionReturns() {
    succeeds("""
        (long ?result) = ROOTworker() { ?result = 6 * 7; }
        () = ROOT() {
          AslTask !task;
          ActionStatus !status;
          long !number = 1;
          String !text = "before";
          !task = async(!number, !text) {
            !number = act:ROOTworker();
            !text = "after";
          }
          if (!number == 1 && !text == "before") { } else { exit(FAIL); }
          (!status, !number, !text) = join(!task);
          if (!number == 42 && !text == "after") { } else { exit(FAIL); }
        }
        """);
  }

  @Test(timeout = 15000)
  public void typedTasksCaptureInputsBeforeParentMutationAndDoNotPublishOnTimeout() {
    succeeds("""
        () = ROOT() {
          AslTask !task;
          ActionStatus !status;
          long !input = 21;
          long !number = 7;
          !task = async(!number) { op:sleep(100); !number = !input * 2; }
          !input = 99;
          (!status, !number) = join(!task, 0);
          if (!number == 7) { } else { exit(FAIL); }
          (!status, !number) = join(!task);
          if (!number == 42 && !input == 99) { } else { exit(FAIL); }
        }
        """);
  }

  @Test(timeout = 15000)
  public void failedTasksDoNotPublishPartialResults() {
    succeeds("""
        () = ROOT() {
          AslTask !task;
          ActionStatus !status;
          long !number = 7;
          !task = async(!number) { !number = 99; exit(FAIL); }
          (!status, !number) = join(!task);
          if (!number == 7) { } else { exit(FAIL); }
        }
        """);
  }

  @Test(timeout = 15000)
  public void taskJoinChecksAllOutputTypesBeforePublishingAnyResult() {
    ExecutionDiagnostic error = fails("""
        () = ROOT() {
          AslTask !task;
          ActionStatus !status;
          long !number = 7;
          String !text = "original";
          long !wrong;
          !task = async(!number, !text) { !number = 42; !text = "new"; }
          (!status, !number, !wrong) = join(!task);
        }
        """);
    assertTrue(error.reason(), error.reason().contains("Cannot assign"));
    assertEquals("7", error.arguments().get("result0"));
  }

  @Test(timeout = 15000)
  public void legacyLongAsyncIdsAndJoinStillWork() {
    succeeds("""
        () = ROOT() {
          long !id;
          ActionStatus !status;
          !id = async { op:sleep(1); }
          !status = join(!id);
        }
        """);
  }

  @Test(timeout = 15000)
  public void scopedModulesResolvePrivateHelpersWithoutGlobalRegistration() {
    String module = """
        (long ?result) = privateEnhancementHelper(long ?value) { ?result = ?value * 2; }
        (long ?result) = entry(long ?value) { ?result = act:privateEnhancementHelper(?value); }
        """;
    succeeds("""
        () = ROOT() {
          CompiledAction !module;
          String !source;
          long !answer;
          !source = MODULE;
          !module = op:compileAsl("private-module.asl", !source, "entry");
          !answer = invoke(!module, ?actor, 21);
          if (!answer == 42) { } else { exit(FAIL); }
        }
        """.replace("MODULE", quote(module)));
    assertNull(Database.getActionDB().getAction("privateEnhancementHelper"));
    assertNull(Database.getActionDB().getAction("entry"));
  }

  @Test(timeout = 15000)
  public void scopedModulesCanRecurseThroughNativeActionContexts() {
    String module = """
        (long ?result) = recurse(long ?value) {
          long !next;
          if (?value == 0) { ?result = 0; }
          else { !next = ?value-1; ?result = act:recurse(!next); ?result = ?result + 1; }
        }
        """;
    succeeds("""
        () = ROOT() {
          CompiledAction !module;
          String !source;
          long !answer;
          !source = MODULE;
          !module = op:compileAsl("recursive-module.asl", !source, "recurse");
          !answer = invoke(!module, ?actor, 4);
          if (!answer == 4) { } else { exit(FAIL); }
        }
        """.replace("MODULE", quote(module)));
  }

  @Test(timeout = 15000)
  public void sourceDiagnosticsExposeOriginalLineAndBindings() {
    ExecutionDiagnostic error = fails("""
        () = ROOT() {
          long !number = 7;
          !number = !number / 0;
        }
        """);
    assertEquals("native-enhancement-test.asl", error.source().source());
    assertEquals(7, error.source().line());
    assertEquals("7", error.arguments().get("!number"));
    assertTrue(error.reason(), error.reason().toLowerCase().contains("zero"));
    assertFalse(error.callStack().isEmpty());
  }

  @Test(timeout = 15000)
  public void numericAssignmentsRejectTruncationAndOverflow() {
    fails("() = ROOT() { int !n; !n = 1 / 2; }");
    fails("() = ROOT() { byte !n; !n = 128; }");
    fails("() = ROOT() { java.lang.Boolean !flag; !flag = 1; }");
  }

  @Test(timeout = 15000)
  public void blockLocalDeclarationsHaveLexicalScopeAndShadowing() {
    succeeds("""
        () = ROOT() {
          long !value = 1;
          if (true) {
            long !value = 2;
            if (!value == 2) { } else { exit(FAIL); }
          } else {
            long !other = 3;
          }
          if (!value == 1) { } else { exit(FAIL); }
        }
        """);
    rejects(() -> run("""
        () = ROOT() {
          if (true) { long !temp = 1; }
          !temp = 2;
        }
        """));
  }

  @Test(timeout = 15000)
  public void standaloneBlocksExecuteWithLexicalScope() {
    succeeds("""
        () = ROOT() {
          long !value = 1;
          block {
            long !value = 2;
            if (!value == 2) { } else { exit(FAIL); }
            block {
              long !inner = !value + 1;
              if (!inner == 3) { } else { exit(FAIL); }
            }
          }
          if (!value == 1) { } else { exit(FAIL); }
        }
        """);
  }

  @Test(timeout = 15000)
  public void forLoopReusesExistingCounterBinding() {
    succeeds("""
        () = ROOT() {
          int !i = 42;
          for (!i=0; !i lt 3; !i++) { }
          if (!i == 3) { } else { exit(FAIL); }
        }
        """);
  }

  @Test(timeout = 15000)
  public void foreachReusesExistingElementBindingWhenPredeclared() {
    succeeds("""
        () = ROOT() {
          java.lang.String !item = "outer";
          java.util.List !items = op:newObject("java.util.ArrayList");
          op:invokeMethod(!items, "add", "a");
          op:invokeMethod(!items, "add", "b");
          foreach (!item : !items) { }
          if (!item == "b") { } else { exit(FAIL); }
        }
        """);
  }

  @Test
  public void newSyntaxSurvivesWriterRoundTrip() {
    var entry = new ActionScriptLanguageParser().parseStrict("roundtrip.asl", null, IMPORTS + """
        () = roundTrip() {
          long !number = 1;
          AslTask !task;
          ActionStatus !status;
          !task = async(!number) { !number = !number + 1; }
          (!status, !number) = join(!task);
          if (!number == 2) { !number = 3; }
        }
        """).get(0);
    String written = new ActionScriptLanguageWriter().writeAction(entry);
    var reparsed = new ActionScriptLanguageParser().parseStrict("written.asl", null, written).get(0);
    assertEquals(entry.getEventSpecs(), reparsed.getEventSpecs());
  }

  @Test(timeout = 15000)
  public void runtimeSourceIsLiteralEvenWhenItsVariablesExistInTheCaller() {
    String module = "(long ?result) = literalEntry(long ?value) { ?result = ?value + 1; }";
    succeeds("""
        () = ROOT() {
          long ?value = 100;
          long ?result = 200;
          CompiledAction !module;
          long !answer;
          !module = op:compileAsl("literal-module.asl", MODULE, "literalEntry");
          !answer = invoke(!module, ?actor, 7);
          if (!answer == 8 && ?value == 100 && ?result == 200) { } else { exit(FAIL); }
        }
        """.replace("MODULE", quote(module)));
  }

  @Test(timeout = 15000)
  public void concurrentModulesKeepIdenticallyNamedHelpersIsolated() {
    String first = "(long ?result) = helper(long ?n) { ?result = ?n * 2; }"
        + "(long ?result) = entry(long ?n) { ?result = act:helper(?n); }";
    String second = first.replace("* 2", "* 3");
    succeeds("""
        () = ROOT() {
          CompiledAction !first;
          CompiledAction !second;
          AslTask !a;
          AslTask !b;
          ActionStatus !status;
          long !left;
          long !right;
          !first = op:compileAsl("first.asl", FIRST, "entry");
          !second = op:compileAsl("second.asl", SECOND, "entry");
          !a = async(!left) { !left = invoke(!first, ?actor, 7); }
          !b = async(!right) { !right = invoke(!second, ?actor, 7); }
          (!status, !left) = join(!a);
          (!status, !right) = join(!b);
          if (!left == 14 && !right == 21) { } else { exit(FAIL); }
        }
        """.replace("FIRST", quote(first)).replace("SECOND", quote(second)));
  }

  @Test(timeout = 15000)
  public void scopedInvocationHonorsExplicitActor() {
    String module = "(edu.tufts.hrilab.fol.Symbol ?result) = entry() { ?result = ?actor; }";
    succeeds("""
        () = ROOT() {
          CompiledAction !module;
          edu.tufts.hrilab.fol.Symbol !actor;
          !module = op:compileAsl("actor.asl", MODULE, "entry");
          !actor = invoke(!module, other:agent);
          if (!actor == other:agent && ?actor == self:agent) { } else { exit(FAIL); }
        }
        """.replace("MODULE", quote(module)));
  }

  @Test(timeout = 15000)
  public void taskCancellationExecutesFinallyWithoutCancellingItsParent() throws Exception {
    long goal = run("""
        () = ROOT() {
          AslTask !task;
          ActionStatus !status;
          java.lang.Boolean !cleaned = false;
          !task = async(!cleaned) {
            try { op:sleep(1000); } finally { !cleaned = true; }
          }
          !status = join(!task);
        }
        """);
    edu.tufts.hrilab.action.execution.AslTask task = null;
    long deadline = System.nanoTime() + java.util.concurrent.TimeUnit.SECONDS.toNanos(5);
    while (task == null && System.nanoTime() < deadline) {
      var root = manager.getGoal(goal).getRootContext();
      if (root != null) {
        for (var child : root.getChildContexts().getChildrenContexts()) {
          if (child instanceof edu.tufts.hrilab.action.execution.ActionContext) {
            Object value = child.getArgumentValue("!task");
            if (value instanceof edu.tufts.hrilab.action.execution.AslTask) {
              task = (edu.tufts.hrilab.action.execution.AslTask) value;
            }
          }
        }
      }
      if (task == null) Thread.sleep(5);
    }
    assertNotNull(task);
    while (!task.isStarted() && System.nanoTime() < deadline) Thread.sleep(5);
    assertTrue(task.isStarted());
    while ((task.getNativeGoal().getCurrentContext() == null
        || !task.getNativeGoal().getCurrentContext().getCommand().equals("sleep"))
        && System.nanoTime() < deadline) Thread.sleep(5);
    assertNotNull(task.getNativeGoal().getCurrentContext());
    assertEquals("sleep", task.getNativeGoal().getCurrentContext().getCommand());
    assertTrue(task.cancel());
    assertEquals(edu.tufts.hrilab.action.ActionStatus.CANCEL, task.await(5000));
    assertEquals(Boolean.TRUE, task.getNativeGoal().getRootContext().readValue("!cleaned"));
    assertEquals(GoalStatus.SUCCEEDED, manager.joinOnGoal(goal, 5000));
  }

  @Test
  public void strictCompilationRejectsMissingEntryAndMalformedSource() {
    rejects(() -> CompiledAction.compile("bad.asl", "() = x() {", "x"));
    rejects(() -> CompiledAction.compile("missing.asl", "() = x() {}", "other"));
    assertEquals("invoke", CompiledAction.compile("legacy-name.asl",
        "() = invoke() {}", "invoke").getEntry().getName());
  }

  @Test
  public void expressionNestingAndUnaryLimitsAreCheckedBeforeParsing() {
    assertEquals(Boolean.TRUE, new AslExpression("(".repeat(32) + "true" + ")".repeat(32)).evaluate(null));
    rejects(() -> new AslExpression("(".repeat(33) + "true" + ")".repeat(33)));
    assertEquals(Boolean.TRUE, new AslExpression("~".repeat(64) + "true").evaluate(null));
    rejects(() -> new AslExpression("~".repeat(65) + "true"));
  }

  @Test
  public void expressionCharacterAndTokenLimitsIncludeTheirExactBoundaries() {
    String literal = "\"" + "a".repeat(16382) + "\"";
    assertEquals("a".repeat(16382), new AslExpression(literal).evaluate(null));
    rejects(() -> new AslExpression(literal + " "));
    assertEquals(new java.math.BigDecimal("1024"),
        new AslExpression("1+".repeat(1023) + "1").evaluate(null));
    rejects(() -> new AslExpression("1+".repeat(1024) + "1"));
  }

  @Test(timeout = 15000)
  public void globallyDefinedActionsDoNotInheritPrivateModuleNames() {
    String module = "(long ?result) = ROOTshared() { ?result = 99; }"
        + "(long ?result) = entry() { ?result = act:ROOTglobal(); }";
    succeeds("""
        (long ?result) = ROOTshared() { ?result = 3; }
        (long ?result) = ROOTglobal() { ?result = act:ROOTshared(); }
        () = ROOT() {
          CompiledAction !module;
          long !answer;
          !module = op:compileAsl("namespace.asl", MODULE, "entry");
          !answer = invoke(!module, ?actor);
          if (!answer == 3) { } else { exit(FAIL); }
        }
        """.replace("MODULE", quote(module)));
  }

  @Test(timeout = 15000)
  public void scopedHelpersRejectExtraArgumentsRatherThanIgnoringThem() {
    String module = "() = helper() {} () = entry() { act:helper(1); }";
    fails("""
        () = ROOT() {
          CompiledAction !module;
          !module = op:compileAsl("wrong-arity.asl", MODULE, "entry");
          invoke(!module, ?actor);
        }
        """.replace("MODULE", quote(module)));
  }

  @Test(timeout = 15000)
  public void distributedExampleExecutesAsANativeGoal() {
    long goal = manager.submitGoal(Factory.createPredicate("exampleNativeAslFeatures(self:agent)"));
    goals.add(goal);
    assertEquals(GoalStatus.SUCCEEDED, manager.joinOnGoal(goal, 5000));
  }

  @Test(timeout = 15000)
  public void legacyAsyncExpressionAssignmentsKeepDirectChildSnapshotSemantics() {
    succeeds("""
        () = ROOT() {
          long !id;
          ActionStatus !status;
          long !number = 1;
          !id = async { !number = 42; }
          !status = join(!id);
          if (!number == 1) { } else { exit(FAIL); }
        }
        """);
  }

  private static void rejects(Runnable operation) {
    try {
      operation.run();
    } catch (IllegalArgumentException expected) {
      return;
    }
    fail("Expected explicit rejection");
  }
}
