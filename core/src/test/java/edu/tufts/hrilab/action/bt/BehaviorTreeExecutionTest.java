/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.junit.Assume;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicBoolean;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.Assert.*;

/**
 * Integration tests: XML -> ASL parser -> real action DB -> Goal Manager ->
 * Action Interpreters. There is deliberately no Java evaluator for control nodes.
 */
public class BehaviorTreeExecutionTest {
  protected static GoalManagerComponent manager;
  private static final AtomicInteger names = new AtomicInteger();
  private static String startupAction;
  private static final String EXAMPLES = "src/main/resources/config/edu/tufts/hrilab/action/bt/";
  protected final List<String> trace = new CopyOnWriteArrayList<>();
  private final Map<String, Boolean> checks = new ConcurrentHashMap<>();
  private final Map<String, CountDownLatch> finish = new ConcurrentHashMap<>();
  private final Map<String, CountDownLatch> started = new ConcurrentHashMap<>();
  private final AtomicInteger attempts = new AtomicInteger();
  private final AtomicInteger beats = new AtomicInteger();
  private final AtomicBoolean scriptFinished = new AtomicBoolean();
  private final AtomicBoolean observedSafe = new AtomicBoolean(true);
  private final List<Long> submitted = new ArrayList<>();
  private java.util.Collection<TRADEServiceInfo> services;

  @BeforeClass
  public static void initialize() {
    initializeManager("edu.tufts.hrilab.action.manager.ExecutionManager");
  }

  protected static void initializeManager(String type) {
    startupAction = "btStartup" + names.incrementAndGet();
    manager = DiarcComponent.createInstance(GoalManagerComponent.class, true, false,
        "-beliefinitfile agents/agents.pl -nomemorymanager -executionManagerType " + type
            + " -asl bt-examples.asl -bt " + EXAMPLES + "reactive_navigation.xml "
            + EXAMPLES + "example-bindings.json " + startupAction);
  }

  @AfterClass
  public static void shutdown() {
    manager.shutdownComponent();
  }

  @Before
  public void register() throws Exception {
    services = TRADE.registerAllServices(this, List.of());
    Database.getInstance().addPrimitiveActions(null);
    await(() -> Database.getActionDB().getAction("btRecord") != null);
  }

  @After
  public void cleanup() throws Exception {
    scriptFinished.set(true);
    finish.values().forEach(CountDownLatch::countDown);
    for (long id : submitted) {
      if (!manager.getGoalStatus(id).isTerminated()) {
        manager.cancelGoal(id);
      }
      assertTrue("A test left a running goal", manager.joinOnGoal(id, 5000).isTerminated());
    }
    for (var entry : Database.getActionDB().getAllActions()) {
      if (services.contains(entry.getServiceInfo())) {
        Database.getInstance().removeActionDBEntry(entry);
      }
    }
    for (var entry : Database.getObserverDB().getObservers(Factory.createPredicate("btObservedSafe(self:agent)"))) {
      if (services.contains(entry.getServiceInfo())) {
        Database.getInstance().removeActionDBEntry(entry);
      }
    }
    TRADE.deregister(this);
  }

  protected long submit(String body) {
    return submitXml(BehaviorTreeConverterTest.xml(body), BehaviorTreeConverterTest.bindings());
  }

  private long submitXml(String xml, BehaviorTreeBindings bindings) {
    String action = "btTest" + names.incrementAndGet();
    new BehaviorTreeConverter(bindings).actions(xml, action).forEach(Database.getInstance()::addActionDBEntry);
    long id = manager.submitGoal(Factory.createPredicate(action + "(self:agent)"));
    return track(id);
  }

  protected long track(long id) {
    submitted.add(id);
    return id;
  }

  protected void succeeds(long id) {
    assertEquals("Trace: " + trace, GoalStatus.SUCCEEDED, manager.joinOnGoal(id, 5000));
  }

  private void fails(long id) {
    assertEquals("Trace: " + trace, GoalStatus.FAILED, manager.joinOnGoal(id, 5000));
  }

  protected void prepare(String label) {
    finish.put(label, new CountDownLatch(1));
    started.put(label, new CountDownLatch(1));
  }

  protected void release(String label) {
    finish.get(label).countDown();
  }

  protected void waitStarted(String label) throws InterruptedException {
    assertTrue("Did not start " + label + "; trace=" + trace,
        started.get(label).await(5, TimeUnit.SECONDS));
  }

  private static void await(java.util.function.BooleanSupplier condition) throws Exception {
    long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(5);
    while (!condition.getAsBoolean() && System.nanoTime() < deadline) {
      Thread.sleep(5);
    }
    assertTrue("Timed out waiting for condition", condition.getAsBoolean());
  }

  private BehaviorTreeExecution execution(long id) {
    edu.tufts.hrilab.action.execution.Context context = manager.getGoal(id).getCurrentContext();
    while (context != null) {
      if (context instanceof edu.tufts.hrilab.action.execution.ArgumentBasedContext
          && ((edu.tufts.hrilab.action.execution.ArgumentBasedContext) context).hasLocalArgument("!execution")) {
        return (BehaviorTreeExecution) context.getArgumentValue("!execution");
      }
      context = context.getParentContext();
    }
    throw new AssertionError("Missing behavior-tree execution frame");
  }

  @Test(timeout = 15000)
  public void remainingBuiltinsExecuteThroughNativeAsl() {
    succeeds(submit("""
        <Sequence>
          <SetBlackboard output_key="source" value="room"/>
          <SetBlackboard output_key="copy" value="{source}"/>
          <ScriptCondition code="copy == 'room'"/>
          <WasEntryUpdated entry="source"/>
          <SkipUnlessUpdated entry="{source}"><Record label="updated"/></SkipUnlessUpdated>
          <Fallback><WasEntryUpdated entry="missing"/><AlwaysSuccess/></Fallback>
          <UnsetBlackboard key="source"/>
          <Fallback><WasEntryUpdated entry="source"/><AlwaysSuccess/></Fallback>
          <Sleep msec="1"/>
          <LoopInt queue="1;2" value="{item}"><Record label="{item}"/></LoopInt>
          <LoopBool queue="true;false" value="{flag}"><Record label="{flag}"/></LoopBool>
          <LoopDouble queue="1.5;2.5" value="{real}"><Record label="{real}"/></LoopDouble>
          <LoopString queue="a;b" value="{text}"><Record label="{text}"/></LoopString>
          <AsyncSequence><Record label="async1"/><Record label="async2"/></AsyncSequence>
          <AsyncFallback><AlwaysFailure/><Record label="asyncFallback"/></AsyncFallback>
          <Fallback>
            <TryCatch><Record label="try"/><AlwaysFailure/><Record label="catch"/></TryCatch>
            <Record label="recovered"/>
          </Fallback>
        </Sequence>
        """));
    assertEquals(List.of("record:updated", "record:1", "record:2", "record:true", "record:false",
        "record:1.5", "record:2.5", "record:a", "record:b", "record:async1", "record:async2",
        "record:asyncFallback", "record:try", "record:catch", "record:recovered"), trace);
  }

  @Test(timeout = 15000)
  public void dynamicScriptsPreserveCallingActorRatherThanRootActor() {
    RootContext root = new RootContext(new ActionConstraints(), null);
    GoalContext goal = new GoalContext(root, null,
        Goal.createGoal(Factory.createPredicate("actorRoot(self:agent)")), ExecutionType.ACT);
    ActionContext parent = new ActionContext(goal, null,
        BehaviorTreeConverter.parse("() = scriptParent() {}").get(0),
        List.of(), List.of(), goal.getGoal().getActor());
    BehaviorTreeExecution frame = new BehaviorTreeExecution(goal, "{}");
    Symbol other = Factory.createSymbol("other:agent");
    var helper = BehaviorTreeConverter.parse("""
        import java.lang.Boolean;
        (Boolean ?answer) = actorScript(edu.tufts.hrilab.action.bt.BehaviorTreeExecution ?execution) {
        }
        """).get(0);
    parent.setStatus(ActionStatus.PROGRESS);
    ActionContext caller = parent.addAction(helper, List.of(frame), List.of(), other);
    assertSame(caller, parent.getNextStep());
    caller.setStatus(ActionStatus.PROGRESS);

    frame.appendScript(parent, java.util.Base64.getEncoder().encodeToString(
        "x := 1".getBytes(java.nio.charset.StandardCharsets.UTF_8)), false);

    assertEquals(other, caller.getChildContexts().get(0).getArgumentValue("?actor"));
  }

  @Test(timeout = 15000)
  public void dynamicScriptsAndGlobalReferencesUseNativeCallFrames() {
    succeeds(submitXml("""
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main"><Sequence>
            <Script code="x := 1"/>
            <SetBlackboard output_key="source" value="x = 4"/>
            <Script code="{source}"/>
            <SetBlackboard output_key="test" value="x == 4"/>
            <ScriptCondition code="{test}"/>
            <SubTree ID="Child"/>
            <ScriptCondition code="x == 7"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="Child"><Sequence>
            <Script code="x := 99; @x += 3"/>
            <ScriptCondition code="x == 99 &amp;&amp; @x == 7"/>
            <Record label="{@x}"/>
          </Sequence></BehaviorTree>
        </root>
        """, BehaviorTreeConverterTest.bindings()));
    assertEquals(List.of("record:7"), trace);
  }

  @Test(timeout = 15000)
  public void updateCountersSurviveHaltsAndDetectSameValueWrites() {
    succeeds(submit("""
        <Sequence>
          <Script code="x := 1; count := 0"/>
          <Repeat num_cycles="3">
            <Sequence>
              <Script code="x = 1"/>
              <SkipUnlessUpdated entry="x"><Script code="count += 1"/></SkipUnlessUpdated>
            </Sequence>
          </Repeat>
          <ScriptCondition code="count == 3"/>
          <ParallelAll>
            <WaitValueUpdate entry="later"><Record label="notified"/></WaitValueUpdate>
            <Sequence><Sleep msec="20"/><Script code="later := 1"/></Sequence>
          </ParallelAll>
        </Sequence>
        """));
    assertEquals(1, Collections.frequency(trace, "record:notified"));
  }

  @Test(timeout = 15000)
  public void tryCatchCleanupRunsOnFailureAndOnHaltWithoutChangingFailureOutcome() throws Exception {
    fails(submit("<TryCatch><AlwaysFailure/><Record label=\"failureCleanup\"/></TryCatch>"));
    prepare("tryRunning");
    long id = submit("""
        <TryCatch catch_on_halt="true">
          <Work label="tryRunning"/><Record label="haltCleanup"/>
        </TryCatch>
        """);
    waitStarted("tryRunning");
    assertTrue(manager.cancelGoal(id));
    assertEquals(GoalStatus.CANCELED, manager.joinOnGoal(id, 5000));
    assertTrue(trace.indexOf("halt:tryRunning") < trace.indexOf("record:haltCleanup"));
    assertEquals(1, Collections.frequency(trace, "record:haltCleanup"));
  }

  @Test(timeout = 15000)
  public void explicitBeliefsPersistButUnmappedVariablesRemainInvocationLocal() {
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings()
        .registerEnum("INSPECT", 2)
        .bindBelief("done", "btSharedDone(self:agent)", "boolean")
        .bindBelief("destination", "btSharedDestination(self:agent,Value)", "String");
    succeeds(submitXml(BehaviorTreeConverterTest.xml("""
        <Sequence>
          <Script code="done = true; destination = 'lab'; local := 42; mode := INSPECT"/>
          <ScriptCondition code="mode == 2"/>
        </Sequence>
        """), bindings));
    succeeds(submitXml(BehaviorTreeConverterTest.xml("""
        <Sequence>
          <ScriptCondition code="done &amp;&amp; destination == 'lab'"/>
          <Script code="done = false"/>
        </Sequence>
        """), bindings));
    succeeds(submitXml(BehaviorTreeConverterTest.xml("<ScriptCondition code=\"!done\"/>"), bindings));
    fails(submitXml(BehaviorTreeConverterTest.xml("<ScriptCondition code=\"local == 42\"/>"), bindings));
  }

  @Test(timeout = 15000)
  public void typedQueuesAreConsumedAndCustomConvertersUseNativeLeafTypes() {
    String pointType = "Point" + names.incrementAndGet();
    BehaviorTreeExecution.registerPortConverter(pointType, java.awt.Point.class, text -> {
      String[] parts = text.split(",");
      if (parts.length != 2) {
        throw new IllegalArgumentException("Expected x,y");
      }
      return new java.awt.Point(Integer.parseInt(parts[0]), Integer.parseInt(parts[1]));
    });
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings();
    var queue = new BehaviorTreeBindings.Leaf("btProduceQueue", true, "action");
    queue.outputs.add(new BehaviorTreeBindings.Port("queue", "Queue<int>"));
    bindings.bind("Queue", queue).bind("Point", new BehaviorTreeBindings.Leaf("btPoint", true, "action",
        new BehaviorTreeBindings.Port("point", pointType)));
    succeeds(submitXml(BehaviorTreeConverterTest.xml("""
        <Sequence>
          <Queue queue="{queue}"/>
          <LoopInt queue="{queue}" value="{item}"><Record label="{item}"/></LoopInt>
          <LoopInt queue="{queue}" value="{item}"><AlwaysFailure/></LoopInt>
          <Repeat num_cycles="2">
            <LoopString queue="a;b" value="{text}"><Record label="{text}"/></LoopString>
          </Repeat>
          <Point point="3,4"/>
        </Sequence>
        """), bindings));
    assertEquals(List.of("record:1", "record:2", "record:a", "record:b", "record:a", "record:b",
        "point:3,4"), trace);
  }

  @Test(timeout = 15000)
  public void dynamicHooksAndFalsePredicatesRetainNativeFailureAndReturnSemantics() {
    succeeds(submit("""
        <Sequence>
          <Script code="done := false"/>
          <SetBlackboard output_key="hook" value="done = true"/>
          <AlwaysSuccess _onSuccess="{hook}"/>
          <ScriptCondition code="done"/>
          <SetBlackboard output_key="test" value="false"/>
          <Fallback><ScriptCondition code="{test}"/><Record label="falsePredicate"/></Fallback>
        </Sequence>
        """));
    assertEquals(List.of("record:falsePredicate"), trace);
    assertNull(Database.getActionDB().getAction("btRuntimeScript"));
    fails(submit("""
        <Sequence><SetBlackboard output_key="source" value="badCall()"/>
          <Script code="{source}"/>
        </Sequence>
        """));
    fails(submit("<LoopInt queue=\"1;bad\" value=\"{v}\"><AlwaysSuccess/></LoopInt>"));
    fails(submit("<Sleep msec=\"-1\"/>"));
  }

  @Test(timeout = 15000)
  public void queueLoopsKeepTheCurrentItemUntilItsNativeChildCompletes() throws Exception {
    prepare("firstItem");
    prepare("secondItem");
    long id = submit("""
        <LoopString queue="firstItem;secondItem" value="{item}">
          <Work label="{item}"/>
        </LoopString>
        """);
    waitStarted("firstItem");
    assertFalse(trace.contains("start:secondItem"));
    release("firstItem");
    waitStarted("secondItem");
    release("secondItem");
    succeeds(id);
    assertEquals(1, Collections.frequency(trace, "start:firstItem"));
    assertEquals(1, Collections.frequency(trace, "start:secondItem"));
  }

  @Test(timeout = 15000)
  public void invalidBeliefWritesDoNotErasePreviouslyStoredSemanticState() {
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings()
        .bindBelief("number", "btSharedNumber(self:agent,Value)", "int");
    succeeds(submitXml(BehaviorTreeConverterTest.xml("<Script code=\"number = 42\"/>"), bindings));
    fails(submitXml(BehaviorTreeConverterTest.xml("<Script code=\"number = 'bad'\"/>"), bindings));
    succeeds(submitXml(BehaviorTreeConverterTest.xml("<ScriptCondition code=\"number == 42\"/>"), bindings));
  }

  @Test(timeout = 15000)
  public void beliefBackedPortsAndNativeAslConditionsAndEffectsShareOneStateMachine() {
    String action = "btSemantic" + names.incrementAndGet();
    BehaviorTreeConverter.parse("() = " + action + """
        () {
          conditions: { pre infer: btNativeReady(?actor); }
          effects: { success infer: btNativeCompleted(?actor); }
          act:btRecord("semantic");
        }
        """).forEach(Database.getInstance()::addActionDBEntry);
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings()
        .bindBelief("ready", "btNativeReady(self:agent)", "boolean")
        .bindBelief("completed", "btNativeCompleted(self:agent)", "boolean")
        .bind("Semantic", new BehaviorTreeBindings.Leaf(action, true, "action"));
    succeeds(submitXml(BehaviorTreeConverterTest.xml("""
        <Sequence><Script code="ready = true"/><Semantic/><ScriptCondition code="completed"/></Sequence>
        """), bindings));
    succeeds(submitXml(BehaviorTreeConverterTest.xml("""
        <Sequence><ScriptCondition code="completed"/><Script code="ready = false"/>
          <Fallback><Semantic/><Record label="preconditionBlocked"/></Fallback>
        </Sequence>
        """), bindings));
    assertEquals(List.of("record:semantic", "record:preconditionBlocked"), trace);
  }

  @Action
  @TRADEService
  public java.util.Deque<Integer> btProduceQueue() {
    return new java.util.ArrayDeque<>(List.of(1, 2));
  }

  @Action
  @TRADEService
  public void btPoint(java.awt.Point point) {
    trace.add("point:" + point.x + "," + point.y);
  }

  @Test(timeout = 15000)
  public void startupAndRuntimeLoadingExecuteTheDistributedExamples() throws Exception {
    succeeds(track(manager.submitGoal(Factory.createPredicate(startupAction + "(self:agent)"))));
    String action = "btInspection" + names.incrementAndGet();
    manager.loadBehaviorTree(EXAMPLES + "inspection.xml", EXAMPLES + "example-bindings.json", action);
    succeeds(track(manager.submitGoal(Factory.createPredicate(action + "(self:agent)"))));
    String recursive = "btRecursiveExample" + names.incrementAndGet();
    manager.loadBehaviorTree(EXAMPLES + "scripted_recursive_inspection.xml",
        EXAMPLES + "example-bindings.json", recursive);
    succeeds(track(manager.submitGoal(Factory.createPredicate(recursive + "(self:agent)"))));
    String complete = "btNativeFeatures" + names.incrementAndGet();
    manager.loadBehaviorTree(EXAMPLES + "native_features.xml",
        EXAMPLES + "native-feature-bindings.json", complete);
    succeeds(track(manager.submitGoal(Factory.createPredicate(complete + "(self:agent)"))));
    try {
      manager.loadBehaviorTree(EXAMPLES + "inspection.xml", EXAMPLES + "example-bindings.json", action);
      fail("Loading duplicate generated action names should fail");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("already exists"));
    }
  }

  @Test(timeout = 15000)
  public void simultaneousInvocationsHaveIndependentBlackboardsAndCancellation() throws Exception {
    String action = "btIsolated" + names.incrementAndGet();
    new BehaviorTreeConverter(BehaviorTreeConverterTest.bindings())
        .actions(BehaviorTreeConverterTest.xml("<Work label=\"{label}\"/>"), action)
        .forEach(Database.getInstance()::addActionDBEntry);
    prepare("one");
    prepare("two");
    long one = track(manager.submitGoal(Factory.createPredicate(action, Factory.createSymbol("robotone:agent"),
        new Symbol("{\"label\":\"one\"}"))));
    long two = track(manager.submitGoal(Factory.createPredicate(action, Factory.createSymbol("robottwo:agent"),
        new Symbol("{\"label\":\"two\"}"))));
    waitStarted("one");
    waitStarted("two");
    assertTrue(manager.cancelGoal(one));
    assertEquals(GoalStatus.CANCELED, manager.joinOnGoal(one, 5000));
    assertEquals(GoalStatus.ACTIVE, manager.getGoalStatus(two));
    release("two");
    succeeds(two);
    assertTrue(trace.contains("halt:one"));
    assertFalse(trace.contains("halt:two"));
  }

  @Test(timeout = 15000)
  public void scriptsAndCompletionHooksExecuteInNativeAsl() {
    succeeds(submit("""
        <Sequence>
          <Script code="x := 2; text := 'hello'; flag := true"/>
          <Script code="x += 3; text += ' world'"/>
          <ScriptCondition code="x == 5 &amp;&amp; text == 'hello world' &amp;&amp; flag"/>
          <AlwaysFailure _successIf="x == 5" _onSuccess="x = 7" _post="x += 1"/>
          <Fallback>
            <AlwaysSuccess _failureIf="x == 8" _onFailure="x = 9" _post="x += 1" _skipIf="true"/>
            <ScriptCondition code="x == 10"/>
          </Fallback>
          <ScriptCondition code="true || false"/>
        </Sequence>
        """));
  }

  @Test(timeout = 15000)
  public void skippedNodesAndRunOncePropagateThroughControls() {
    succeeds(submit("""
        <Sequence>
          <Script code="count := 0"/>
          <Repeat num_cycles="3">
            <Sequence>
              <AlwaysFailure _skipIf="true"/>
              <RunOnce><Script code="count += 1"/></RunOnce>
              <Record label="cycle"/>
            </Sequence>
          </Repeat>
          <ScriptCondition code="count == 1"/>
          <Inverter><ScriptCondition code="false"/></Inverter>
          <ForceSuccess><AlwaysFailure _skipIf="true"/></ForceSuccess>
        </Sequence>
        """));
    assertEquals(3, Collections.frequency(trace, "record:cycle"));
    succeeds(submit("<Fallback><AlwaysSuccess _skipIf=\"true\"/><Record label=\"fallback\"/></Fallback>"));
    succeeds(submit("<ParallelAll><AlwaysFailure _skipIf=\"true\"/><AlwaysSuccess/></ParallelAll>"));
    succeeds(submit("<Parallel success_count=\"-1\"><AlwaysFailure _skipIf=\"true\"/><AlwaysSuccess/></Parallel>"));
  }

  @Test(timeout = 15000)
  public void conditionalAndSwitchControlsChooseBranches() {
    succeeds(submit("""
        <Sequence>
          <Script code="mode := 2"/>
          <IfThenElse><ScriptCondition code="mode == 1"/><AlwaysFailure/><Record label="else"/></IfThenElse>
          <Switch3 variable="{mode}" case_1="1" case_2="2" case_3="3">
            <AlwaysFailure/><Record label="two"/><AlwaysFailure/><AlwaysFailure/>
          </Switch3>
          <WhileDoElse><ScriptCondition code="true"/><Record label="then"/><AlwaysFailure/></WhileDoElse>
          <Precondition if="mode == 2"><Record label="guarded"/></Precondition>
          <RunOnce then_skip="false"><Record label="once"/></RunOnce>
        </Sequence>
        """));
    assertEquals(List.of("record:else", "record:two", "record:then", "record:guarded", "record:once"), trace);
  }

  @Test(timeout = 15000)
  public void switchChangesHaltTheOldBranchBeforeStartingTheNewBranch() throws Exception {
    prepare("first");
    prepare("second");
    long id = submit("""
        <Sequence>
          <Script code="mode := 'first'"/>
          <Switch2 variable="{mode}" case_1="first" case_2="second">
            <Work label="first"/><Work label="second"/><AlwaysFailure/>
          </Switch2>
        </Sequence>
        """);
    waitStarted("first");
    execution(id).scriptAssign("mode", "=", "second");
    waitStarted("second");
    assertTrue(trace.indexOf("halt:first") < trace.indexOf("start:second"));
    assertEquals(1, Collections.frequency(trace, "halt:first"));
    release("second");
    succeeds(id);
  }

  @Test(timeout = 15000)
  public void ifThenElseRetainsItsConditionDecisionWhileTheBranchRuns() throws Exception {
    prepare("retained");
    checks.put("guard", true);
    long id = submit("""
        <IfThenElse>
          <Check label="guard"/><Work label="retained"/><AlwaysFailure/>
        </IfThenElse>
        """);
    waitStarted("retained");
    checks.put("guard", false);
    release("retained");
    succeeds(id);
    assertFalse(trace.contains("halt:retained"));
  }

  @Test(timeout = 15000)
  public void whileHookHaltsOnlyTheRunningBranchAndRunsHaltScript() throws Exception {
    prepare("guarded");
    checks.put("guard", true);
    long id = submit("""
        <Sequence>
          <Script code="halted := false"/>
          <Fallback>
            <ReactiveSequence>
              <Check label="guard"/>
              <Work label="guarded" _onHalted="halted = true"/>
            </ReactiveSequence>
            <ScriptCondition code="halted"/>
          </Fallback>
        </Sequence>
        """);
    waitStarted("guarded");
    checks.put("guard", false);
    succeeds(id);
    assertTrue(trace.contains("halt:guarded"));
  }

  @Test(timeout = 15000)
  public void whilePreconditionRechecksButFailureIfDoesNotRecheckRunningLeaves() throws Exception {
    prepare("watched");
    long id = submit("""
        <Sequence>
          <Script code="enabled := true; halted := false"/>
          <ParallelAll>
            <Sequence><Delay delay_msec="150"><Script code="enabled = false"/></Delay></Sequence>
            <Work label="watched" _while="enabled" _onHalted="halted = true"/>
          </ParallelAll>
          <ScriptCondition code="halted"/>
        </Sequence>
        """);
    waitStarted("watched");
    succeeds(id);
    assertTrue(trace.contains("halt:watched"));

    prepare("stable");
    id = submit("""
        <Sequence>
          <Script code="fail := false"/>
          <ParallelAll>
            <Delay delay_msec="150"><Script code="fail = true"/></Delay>
            <Work label="stable" _failureIf="fail"/>
          </ParallelAll>
        </Sequence>
        """);
    waitStarted("stable");
    Thread.sleep(250);
    assertEquals(GoalStatus.ACTIVE, manager.getGoalStatus(id));
    release("stable");
    succeeds(id);
    assertFalse(trace.contains("halt:stable"));
  }

  @Test(timeout = 15000)
  public void recursiveSubtreesUseIsolatedCallFramesAndReturnBindings() {
    succeeds(submitXml("""
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main"><Sequence>
            <Script code="n := 3"/>
            <SubTree ID="Recurse" depth="{n}"/>
            <ScriptCondition code="n == 3"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="Recurse"><IfThenElse>
            <ScriptCondition code="depth == 0"/>
            <Record label="base"/>
            <Sequence>
              <Record label="enter"/>
              <Script code="next := depth - 1"/>
              <SubTree ID="Recurse" depth="{next}"/>
              <Record label="leave"/>
            </Sequence>
          </IfThenElse></BehaviorTree>
        </root>
        """, BehaviorTreeConverterTest.bindings()));
    assertEquals(List.of("record:enter", "record:enter", "record:enter", "record:base",
        "record:leave", "record:leave", "record:leave"), trace);
  }

  @Test(timeout = 15000)
  public void recursiveCancellationRunsHaltHooksFromLeafToRoot() throws Exception {
    prepare("bottom");
    long id = submitXml("""
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main"><Sequence _onHalted="events += 'root'">
            <Script code="n := 2; events := ''"/>
            <SubTree ID="Recurse" depth="{n}" events="{events}"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="Recurse"><IfThenElse>
            <ScriptCondition code="depth == 0"/>
            <Work label="bottom" _onHalted="events += 'leaf'"/>
            <Sequence>
              <Script code="next := depth - 1"/>
              <SubTree ID="Recurse" depth="{next}" events="{events}"/>
            </Sequence>
          </IfThenElse></BehaviorTree>
        </root>
        """, BehaviorTreeConverterTest.bindings());
    waitStarted("bottom");
    BehaviorTreeExecution frame = execution(id);
    assertTrue(manager.cancelGoal(id));
    assertEquals(GoalStatus.CANCELED, manager.joinOnGoal(id, 5000));
    assertEquals("leafroot", frame.scriptRead("events"));
    assertEquals(1, Collections.frequency(trace, "halt:bottom"));
  }

  @Test(timeout = 15000)
  public void nativeAslFinallyRunsBeforeCancellationCompletes() throws Exception {
    prepare("nativeFinally");
    String action = "btFinally" + names.incrementAndGet();
    BehaviorTreeConverter.parse("() = " + action + """
        () {
          try {
            if (op:equals(true, true)) {
              act:btWork("nativeFinally");
            }
          } finally {
            act:btRecord("finallyCompleted");
          }
        }
        """).forEach(Database.getInstance()::addActionDBEntry);
    long id = track(manager.submitGoal(Factory.createPredicate(action + "(self:agent)")));
    waitStarted("nativeFinally");
    assertTrue(manager.cancelGoal(id));
    assertEquals(GoalStatus.CANCELED, manager.joinOnGoal(id, 5000));
    assertEquals(List.of("start:nativeFinally", "halt:nativeFinally", "end:nativeFinally",
        "record:finallyCompleted"), trace);
  }

  @Test(timeout = 15000)
  public void mutuallyRecursiveSubtreesTerminateAndPreserveLocalScopes() {
    succeeds(submitXml("""
        <root BTCPP_format="4" main_tree_to_execute="A">
          <BehaviorTree ID="A"><Sequence>
            <Script code="n := 3"/>
            <SubTree ID="B" n="{n}"/>
            <ScriptCondition code="n == 3"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="B"><IfThenElse>
            <ScriptCondition code="n == 0"/><Record label="base"/>
            <Sequence><Record label="B"/><Script code="next := n - 1"/>
              <SubTree ID="C" n="{next}"/>
            </Sequence>
          </IfThenElse></BehaviorTree>
          <BehaviorTree ID="C"><IfThenElse>
            <ScriptCondition code="n == 0"/><Record label="base"/>
            <Sequence><Record label="C"/><Script code="next := n - 1"/>
              <SubTree ID="B" n="{next}"/>
            </Sequence>
          </IfThenElse></BehaviorTree>
        </root>
        """, BehaviorTreeConverterTest.bindings()));
    assertEquals(List.of("record:B", "record:C", "record:B", "record:base"), trace);
  }

  @Test(timeout = 15000)
  public void sequenceAndFallbackShortCircuitInSourceOrder() {
    checks.put("bad", false);
    succeeds(submit("""
        <Fallback>
          <Sequence><Check label="bad"/><Record label="never"/></Sequence>
          <Record label="alternative"/>
          <Record label="alsoNever"/>
        </Fallback>
        """));
    assertEquals(List.of("check:bad", "record:alternative"), trace);
  }

  @Test(timeout = 15000)
  public void ordinarySequenceDoesNotRecheckCompletedCondition() throws Exception {
    checks.put("battery", true);
    prepare("move");
    long id = submit("<Sequence><Check label=\"battery\"/><Work label=\"move\"/></Sequence>");
    waitStarted("move");
    checks.put("battery", false);
    Thread.sleep(75);
    assertEquals(GoalStatus.ACTIVE, manager.getGoalStatus(id));
    assertEquals(1, Collections.frequency(trace, "check:battery"));
    finish.get("move").countDown();
    succeeds(id);
    assertEquals(1, Collections.frequency(trace, "start:move"));
  }

  @Test(timeout = 15000)
  public void reactiveSequenceRechecksAndHaltsRunningAction() throws Exception {
    checks.put("battery", true);
    prepare("move");
    long id = submit("<ReactiveSequence><Check label=\"battery\"/><Work label=\"move\"/></ReactiveSequence>");
    waitStarted("move");
    await(() -> Collections.frequency(trace, "check:battery") > 1);
    checks.put("battery", false);
    fails(id);
    assertEquals(1, Collections.frequency(trace, "start:move"));
    assertTrue(trace.contains("halt:move"));
  }

  @Test(timeout = 15000)
  public void reactiveFallbackPreemptsLowerPriorityBranch() throws Exception {
    checks.put("urgent", false);
    prepare("routine");
    long id = submit("<ReactiveFallback><Check label=\"urgent\"/><Work label=\"routine\"/></ReactiveFallback>");
    waitStarted("routine");
    checks.put("urgent", true);
    succeeds(id);
    assertTrue(trace.contains("halt:routine"));
  }

  @Test(timeout = 15000)
  public void parallelRunsConcurrentlyAndCancelsAfterThreshold() throws Exception {
    prepare("one");
    prepare("two");
    long id = submit("<Parallel success_count=\"1\" failure_count=\"1\">"
        + "<Work label=\"one\"/><Work label=\"two\"/></Parallel>");
    waitStarted("one");
    waitStarted("two");
    finish.get("one").countDown();
    succeeds(id);
    assertTrue(trace.contains("halt:two"));
  }

  @Test(timeout = 15000)
  public void parallelEarlyThresholdDoesNotStartLaterChild() {
    succeeds(submit("<Parallel success_count=\"1\"><AlwaysSuccess/><Record label=\"never\"/></Parallel>"));
    assertTrue(trace.isEmpty());
  }

  @Test(timeout = 15000)
  public void timeoutHaltsAndCancellationCleansUp() throws Exception {
    prepare("slow");
    long id = submit("<Timeout msec=\"100\"><Work label=\"slow\"/></Timeout>");
    waitStarted("slow");
    fails(id);
    assertTrue(trace.contains("halt:slow"));
    prepare("cancel");
    id = submit("<Work label=\"cancel\"/>");
    waitStarted("cancel");
    assertTrue(manager.cancelGoal(id));
    assertEquals(GoalStatus.CANCELED, manager.joinOnGoal(id, 5000));
    assertTrue(trace.contains("halt:cancel"));
  }

  @Test(timeout = 15000)
  public void retriesAndRepeatsUseNativeScripts() {
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings()
        .bind("Attempt", new BehaviorTreeBindings.Leaf("btAttempt", true, "boolean"));
    succeeds(submitXml(BehaviorTreeConverterTest.xml("""
        <Sequence>
          <RetryUntilSuccessful num_attempts="3"><Attempt/></RetryUntilSuccessful>
          <Repeat num_cycles="2"><Record label="repeat"/></Repeat>
        </Sequence>
        """), bindings));
    assertEquals(3, attempts.get());
    assertEquals(2, Collections.frequency(trace, "record:repeat"));
  }

  @Test(timeout = 15000)
  public void sequenceWithMemoryRetainsFailedChildAcrossRetries() {
    for (String kind : List.of("Sequence", "SequenceWithMemory")) {
      attempts.set(0);
      trace.clear();
      BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings()
          .bind("Attempt", new BehaviorTreeBindings.Leaf("btAttempt", true, "boolean"));
      succeeds(submitXml(BehaviorTreeConverterTest.xml("<RetryUntilSuccessful num_attempts=\"3\"><"
          + kind + "><Record label=\"first\"/><Attempt/></" + kind
          + "></RetryUntilSuccessful>"), bindings));
      assertEquals(kind, kind.equals("Sequence") ? 3 : 1, Collections.frequency(trace, "record:first"));
    }
  }

  @Test(timeout = 15000)
  public void zeroTimeoutDoesNotDisableTheChildAndDelayWaits() {
    succeeds(submit("<Timeout msec=\"0\"><Record label=\"noTimeout\"/></Timeout>"));
    long before = System.nanoTime();
    succeeds(submit("<Delay delay_msec=\"50\"><Record label=\"delayed\"/></Delay>"));
    assertTrue(TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - before) >= 50);
    assertEquals(List.of("record:noTimeout", "record:delayed"), trace);
  }

  @Test(timeout = 15000)
  public void suspensionAndResumptionRetainNativeScriptProgress() throws Exception {
    String work = "btSuspendable" + names.incrementAndGet();
    BehaviorTreeConverter.parse("() = " + work + "() { act:btMarkStart(); "
        + "while (~tsc:btScriptFinished()) { act:btBeat(); op:sleep(5); } }")
        .forEach(Database.getInstance()::addActionDBEntry);
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings()
        .bind("NativeScript", new BehaviorTreeBindings.Leaf(work, false, "action"));
    long id = submitXml(BehaviorTreeConverterTest.xml("<NativeScript/>"), bindings);
    await(() -> beats.get() >= 2);
    assertTrue(manager.suspendGoal(id));
    await(() -> manager.getGoalStatus(id) == GoalStatus.SUSPENDED);
    int count = beats.get();
    Thread.sleep(50);
    assertEquals(count, beats.get());
    assertTrue(manager.resumeGoal(id));
    await(() -> beats.get() > count);
    scriptFinished.set(true);
    succeeds(id);
    assertEquals(1, Collections.frequency(trace, "scriptStart"));
  }

  @Test(timeout = 15000)
  public void existingOverallObserversAndPostconditionVerificationArePreserved() throws Exception {
    String monitored = "btMonitored" + names.incrementAndGet();
    BehaviorTreeConverter.parse("() = " + monitored + "() { "
        + "conditions: { overall obs: btObservedSafe(?actor); } "
        + "while (~tsc:btScriptFinished()) { act:btBeat(); op:sleep(5); } }")
        .forEach(Database.getInstance()::addActionDBEntry);
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings()
        .bind("Monitored", new BehaviorTreeBindings.Leaf(monitored, false, "action"));
    long id = submitXml(BehaviorTreeConverterTest.xml(
        "<Fallback><Monitored/><Record label=\"overallRecovery\"/></Fallback>"), bindings);
    await(() -> beats.get() >= 2);
    observedSafe.set(false);
    succeeds(id);
    assertTrue(trace.contains("record:overallRecovery"));

    String post = "btPost" + names.incrementAndGet();
    BehaviorTreeConverter.parse("() = " + post + "() { "
        + "effects: { success obs: btObservedSafe(?actor); } act:btRecord(body); }")
        .forEach(Database.getInstance()::addActionDBEntry);
    bindings = BehaviorTreeConverterTest.bindings()
        .bind("Post", new BehaviorTreeBindings.Leaf(post, true, "action"));
    succeeds(submitXml(BehaviorTreeConverterTest.xml(
        "<Fallback><Post/><Record label=\"postRecovery\"/></Fallback>"), bindings));
    assertTrue(trace.contains("record:postRecovery"));
  }

  @Test(timeout = 60000)
  public void matchesPinnedBehaviorTreeCppObservableTraces() throws Exception {
    String reference = System.getenv("DIARC_BT_REFERENCE");
    Assume.assumeTrue(reference != null && !reference.isBlank());
    Map<String, String> scenarios = new java.util.LinkedHashMap<>();
    scenarios.put("fallback", "<Fallback><Check label=\"bad\"/><Record label=\"alternative\"/>"
        + "<Record label=\"never\"/></Fallback>");
    scenarios.put("sequence", "<Sequence><Check label=\"battery\"/><Work label=\"move\"/></Sequence>");
    scenarios.put("reactive-sequence",
        "<ReactiveSequence><Check label=\"battery\"/><Work label=\"move\"/></ReactiveSequence>");
    scenarios.put("reactive-fallback",
        "<ReactiveFallback><Check label=\"urgent\"/><Work label=\"routine\"/></ReactiveFallback>");
    scenarios.put("parallel", "<Parallel success_count=\"1\"><Work label=\"one\"/>"
        + "<Work label=\"two\"/></Parallel>");
    scenarios.put("timeout", "<Timeout msec=\"100\"><Work label=\"slow\"/></Timeout>");
    scenarios.put("retry", "<RetryUntilSuccessful num_attempts=\"3\"><Attempt/></RetryUntilSuccessful>");
    scenarios.put("memory", "<RetryUntilSuccessful num_attempts=\"3\"><SequenceWithMemory>"
        + "<Record label=\"first\"/><Attempt/></SequenceWithMemory></RetryUntilSuccessful>");
    scenarios.put("repeat", "<Repeat num_cycles=\"3\"><Record label=\"repeated\"/></Repeat>");
    scenarios.put("inverter", "<Inverter><Check label=\"bad\"/></Inverter>");
    scenarios.put("force-success", "<ForceSuccess><AlwaysFailure/></ForceSuccess>");
    scenarios.put("force-failure", "<ForceFailure><AlwaysSuccess/></ForceFailure>");
    scenarios.put("delay", "<Delay delay_msec=\"30\"><Record label=\"delayed\"/></Delay>");
    scenarios.put("keep-running", "<KeepRunningUntilFailure><AlwaysFailure/></KeepRunningUntilFailure>");
    scenarios.put("script-hooks", """
        <Sequence>
          <Script code="x := 2; label := 'script'"/>
          <AlwaysFailure _successIf="x == 2" _onSuccess="x += 1" _post="x *= 2"/>
          <ScriptCondition code="x == 6"/>
          <Record label="{label}"/>
        </Sequence>
        """);
    scenarios.put("script-operators", """
        <Sequence>
          <Script code="a := 0; false &amp;&amp; (a = 1); true || (a = 0)"/>
          <ScriptCondition code="a == 0"/>
          <Script code="1 &gt; 2 &lt; (a = 3); true ? (a = 4) : (a = 5); label := 'v' .. 4"/>
          <ScriptCondition code="a == 4 &amp;&amp; (3 &amp; 1) == 1 &amp;&amp; (~1) == -2"/>
          <ScriptCondition code="0.1 + 0.2 == 0.3 &amp;&amp; 'true' == 1"/>
          <Record label="{label}"/>
        </Sequence>
        """);
    scenarios.put("skipped", """
        <Fallback>
          <Sequence><AlwaysFailure _skipIf="true"/><AlwaysSuccess _skipIf="true"/></Sequence>
          <Record label="afterSkipped"/>
        </Fallback>
        """);
    scenarios.put("if-then-else", """
        <IfThenElse><ScriptCondition code="false"/><AlwaysFailure/><Record label="else"/></IfThenElse>
        """);
    scenarios.put("switch", """
        <Sequence><Script code="mode := 2"/>
          <Switch3 variable="{mode}" case_1="1" case_2="2" case_3="3">
            <AlwaysFailure/><Record label="two"/><AlwaysFailure/><AlwaysFailure/>
          </Switch3>
        </Sequence>
        """);
    scenarios.put("run-once", """
        <Repeat num_cycles="3"><Sequence>
          <RunOnce><Record label="once"/></RunOnce><Record label="cycle"/>
        </Sequence></Repeat>
        """);
    scenarios.put("parallel-all", """
        <ParallelAll><AlwaysFailure/><Record label="notAbandoned"/></ParallelAll>
        """);
    scenarios.put("precondition", """
        <Precondition if="true"><Record label="guarded"/></Precondition>
        """);
    scenarios.put("while-do-else", """
        <WhileDoElse><Check label="urgent"/><Record label="urgent"/><Work label="routine"/></WhileDoElse>
        """);
    scenarios.put("try-catch", """
        <TryCatch><Record label="try"/><AlwaysFailure/><Record label="catch"/></TryCatch>
        """);
    scenarios.put("try-catch-success", """
        <TryCatch><Record label="try"/><AlwaysSuccess/><Record label="unused"/></TryCatch>
        """);
    scenarios.put("try-catch-halt", """
        <Timeout msec="30"><TryCatch catch_on_halt="true">
          <Work label="slow"/><Record label="cleanup"/>
        </TryCatch></Timeout>
        """);
    scenarios.put("loop-int", """
        <LoopInt queue="1;2" value="{v}"><Sequence>
          <Script code="label := 'v' .. v"/><Record label="{label}"/>
        </Sequence></LoopInt>
        """);
    scenarios.put("loop-bool", "<LoopBool queue=\"true;false\" value=\"{v}\"><ScriptCondition code=\"v == v\"/></LoopBool>");
    scenarios.put("loop-double", """
        <LoopDouble queue="1.5;2.5" value="{v}"><Sequence>
          <Script code="label := 'v' .. v"/><Record label="{label}"/>
        </Sequence></LoopDouble>
        """);
    scenarios.put("loop-string", "<LoopString queue=\"a;b\" value=\"{v}\"><Record label=\"{v}\"/></LoopString>");
    scenarios.put("blackboard", """
        <Sequence><SetBlackboard output_key="x" value="room"/>
          <Record label="{x}"/>
          <SetBlackboard output_key="y" value="{x}"/>
          <Record label="{y}"/><UnsetBlackboard key="x"/>
          <Fallback><WasEntryUpdated entry="x"/><Record label="missing"/></Fallback>
        </Sequence>
        """);
    scenarios.put("sleep", "<Sequence><Sleep msec=\"2\"/><Record label=\"awake\"/></Sequence>");
    scenarios.put("async-sequence", """
        <AsyncSequence><Record label="first"/><Record label="second"/></AsyncSequence>
        """);
    scenarios.put("async-fallback", """
        <AsyncFallback><AlwaysFailure/><Record label="second"/></AsyncFallback>
        """);
    scenarios.put("update-gates", """
        <Sequence><Script code="x := 1"/>
          <Repeat num_cycles="3"><Sequence><Script code="x = 1"/>
            <WasEntryUpdated entry="x"/>
            <SkipUnlessUpdated entry="{x}"><Record label="updated"/></SkipUnlessUpdated>
          </Sequence></Repeat>
          <ParallelAll>
            <WaitValueUpdate entry="next"><Record label="later"/></WaitValueUpdate>
            <Sequence><Sleep msec="5"/><Script code="next := 1"/></Sequence>
          </ParallelAll>
        </Sequence>
        """);
    scenarios.put("dynamic-script", """
        <Sequence><SetBlackboard output_key="source" value="x := 4"/>
          <Script code="{source}"/><ScriptCondition code="x == 4"/>
          <SetBlackboard output_key="source" value="x = 7"/>
          <Script code="{source}"/><ScriptCondition code="x == 7"/>
          <Record label="dynamic"/>
        </Sequence>
        """);
    scenarios.put("enums", """
        <Sequence><Script code="mode := INSPECT"/>
          <ScriptCondition code="mode == 'INSPECT'"/>
          <Switch2 variable="{mode}" case_1="INSPECT" case_2="3">
            <Record label="enum"/><AlwaysFailure/><AlwaysFailure/>
          </Switch2>
        </Sequence>
        """);
    scenarios.put("global-blackboard", """
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main"><Sequence><Script code="x := 1"/>
            <SubTree ID="Child"/><ScriptCondition code="x == 7"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="Child"><Sequence><Script code="x := 99; @x += 6"/>
            <ScriptCondition code="x == 99 &amp;&amp; @x == 7"/>
            <Record label="global"/>
          </Sequence></BehaviorTree>
        </root>
        """);
    scenarios.put("unset-remapped-entry", """
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main"><Sequence><Script code="x := 7"/>
            <SubTree ID="Child" input="{x}"/>
            <ScriptCondition code="x == 7"/><Record label="retained"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="Child"><UnsetBlackboard key="input"/></BehaviorTree>
        </root>
        """);
    scenarios.put("async-sequence-yield", """
        <ParallelAll>
          <AsyncSequence><Record label="first"/><Record label="second"/></AsyncSequence>
          <Record label="between"/>
        </ParallelAll>
        """);
    scenarios.put("async-fallback-yield", """
        <ParallelAll><AsyncFallback>
          <Sequence><Record label="first"/><AlwaysFailure/></Sequence>
          <Record label="second"/>
        </AsyncFallback><Record label="between"/></ParallelAll>
        """);
    scenarios.put("entry-recreated", """
        <Sequence><Script code="x := 1"/>
          <Repeat num_cycles="2"><Sequence>
            <WasEntryUpdated entry="x"/><UnsetBlackboard key="x"/>
            <Script code="x := 1"/><Record label="recreated"/>
          </Sequence></Repeat>
        </Sequence>
        """);
    scenarios.put("try-catch-running", """
        <TryCatch><AlwaysFailure/><Work label="cleanup"/></TryCatch>
        """);
    for (Map.Entry<String, String> scenario : scenarios.entrySet()) {
      trace.clear();
      checks.put("battery", true);
      checks.put("urgent", false);
      checks.put("bad", false);
      String xml = scenario.getValue().stripLeading().startsWith("<root") ?
          scenario.getValue() : BehaviorTreeConverterTest.xml(scenario.getValue());
      Path input = Files.createTempFile("diarc-bt-reference-", ".xml");
      List<String> expected;
      try {
        Files.writeString(input, xml);
        Process engine = new ProcessBuilder(reference, scenario.getKey(), input.toString())
            .redirectError(ProcessBuilder.Redirect.INHERIT).start();
        assertTrue("Reference engine timed out", engine.waitFor(5, TimeUnit.SECONDS));
        expected = new String(engine.getInputStream().readAllBytes(), java.nio.charset.StandardCharsets.UTF_8)
            .lines().filter(line -> !line.isBlank()).toList();
        assertEquals("Reference engine failed: " + scenario.getKey(), 0, engine.exitValue());
      } finally {
        Files.deleteIfExists(input);
      }
      for (String name : List.of("move", "routine", "one", "two", "slow", "cleanup")) {
        prepare(name);
      }
      attempts.set(0);
      long id = submitXml(xml, BehaviorTreeConverterTest.bindings()
          .registerEnum("INSPECT", 2)
          .bind("Attempt", new BehaviorTreeBindings.Leaf("btAttempt", true, "boolean")));
      switch (scenario.getKey()) {
        case "sequence":
          waitStarted("move");
          checks.put("battery", false);
          finish.get("move").countDown();
          break;
        case "reactive-sequence":
          waitStarted("move");
          checks.put("battery", false);
          break;
        case "reactive-fallback": case "while-do-else":
          waitStarted("routine");
          checks.put("urgent", true);
          break;
        case "parallel":
          waitStarted("one");
          waitStarted("two");
          finish.get("one").countDown();
          break;
        case "try-catch-running":
          waitStarted("cleanup");
          release("cleanup");
          break;
        default: break;
      }
      GoalStatus status = manager.joinOnGoal(id, 5000);
      assertTrue("DIARC tree did not terminate: " + scenario.getKey() + " trace=" + trace, status.isTerminated());
      List<String> actual = new ArrayList<>(trace.stream()
          .filter(event -> event.startsWith("record:") || event.startsWith("start:") || event.startsWith("halt:"))
          .toList());
      if (scenario.getKey().equals("parallel")) {
        // Worker-thread entry order is not a BT ordering guarantee.
        actual.sort(java.util.Comparator.comparingInt(
            (String event) -> event.startsWith("start:") ? 0 : 1).thenComparing(event -> event));
        expected = new ArrayList<>(expected);
        String expectedRoot = expected.remove(expected.size() - 1);
        expected.sort(java.util.Comparator.comparingInt(
            (String event) -> event.startsWith("start:") ? 0 : 1).thenComparing(event -> event));
        expected.add(expectedRoot);
      }
      actual.add("root:" + (status == GoalStatus.SUCCEEDED ? "SUCCESS" : "FAILURE"));
      assertEquals(scenario.getKey(), expected, actual);
    }
  }

  @Test(timeout = 15000)
  public void typedOutputsAndSubtreeRemapsAliasTheSameBlackboard() {
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings();
    BehaviorTreeBindings.Leaf produce = new BehaviorTreeBindings.Leaf("btProduce", true, "action");
    produce.outputs.add(new BehaviorTreeBindings.Port("value", "Symbol"));
    bindings.bind("Produce", produce);
    succeeds(submitXml("""
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main">
            <Sequence>
              <SubTree ID="Child" result="{destination}"/>
              <Record label="{destination}"/>
            </Sequence>
          </BehaviorTree>
          <BehaviorTree ID="Child"><Produce value="{result}"/></BehaviorTree>
        </root>
        """, bindings));
    assertEquals(List.of("record:kitchen"), trace);
  }

  @Test(timeout = 15000)
  public void subtreeLiteralInputsRemainWritableAndInstancesStayIsolated() {
    BehaviorTreeBindings bindings = BehaviorTreeConverterTest.bindings();
    BehaviorTreeBindings.Leaf produce = new BehaviorTreeBindings.Leaf("btProduce", true, "action");
    produce.outputs.add(new BehaviorTreeBindings.Port("value", "Symbol"));
    bindings.bind("Produce", produce);
    succeeds(submitXml("""
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <BehaviorTree ID="Main"><Sequence>
            <SubTree ID="Child" value="first"/>
            <SubTree ID="Child" value="second"/>
          </Sequence></BehaviorTree>
          <BehaviorTree ID="Child"><Sequence>
            <Record label="{value}"/>
            <Produce value="{value}"/>
            <Record label="{value}"/>
          </Sequence></BehaviorTree>
        </root>
        """, bindings));
    assertEquals(List.of("record:first", "record:kitchen", "record:second", "record:kitchen"), trace);
  }

  @Test(timeout = 15000)
  public void canBeCalledFromAnExistingAslScript() {
    String tree = "btEmbedded" + names.incrementAndGet();
    new BehaviorTreeConverter(BehaviorTreeConverterTest.bindings())
        .actions(BehaviorTreeConverterTest.xml("<Record label=\"tree\"/>"), tree)
        .forEach(Database.getInstance()::addActionDBEntry);
    String wrapper = "btWrapper" + names.incrementAndGet();
    BehaviorTreeConverter.parse("() = " + wrapper + "() { act:btRecord(before); act:" + tree
        + "(); act:btRecord(after); }").forEach(Database.getInstance()::addActionDBEntry);
    long id = manager.submitGoal(Factory.createPredicate(wrapper + "(self:agent)"));
    submitted.add(id);
    succeeds(id);
    assertEquals(List.of("record:before", "record:tree", "record:after"), trace);
  }

  @Action @TRADEService
  public void btRecord(Symbol label) {
    trace.add("record:" + label.getName());
  }

  @Action @TRADEService
  public boolean btCheck(Symbol label) {
    trace.add("check:" + label.getName());
    return checks.getOrDefault(label.getName(), false);
  }

  @Action @TRADEService
  public boolean btAttempt() {
    return attempts.incrementAndGet() >= 3;
  }

  @Action @TRADEService
  public Symbol btProduce() {
    return Factory.createSymbol("kitchen");
  }

  @Action @TRADEService
  public void btMarkStart() {
    trace.add("scriptStart");
  }

  @Action @TRADEService
  public void btBeat() {
    beats.incrementAndGet();
  }

  @TRADEService
  public boolean btScriptFinished() {
    return scriptFinished.get();
  }

  @TRADEService
  @Observes({"btObservedSafe(?actor)"})
  public List<Map<Variable, Symbol>> btObserveSafe(Symbol actor, Term state) {
    return observedSafe.get() ? List.of(new java.util.HashMap<>()) : List.of();
  }

  @Action @TRADEService
  @OnInterrupt(onCancelServiceCall = "btStop(?label)")
  public void btWork(Symbol label) throws InterruptedException {
    String name = label.getName();
    trace.add("start:" + name);
    started.get(name).countDown();
    assertTrue("Test work was never released", finish.get(name).await(10, TimeUnit.SECONDS));
    trace.add("end:" + name);
  }

  @TRADEService
  public void btStop(Symbol label) {
    trace.add("halt:" + label.getName());
    finish.get(label.getName()).countDown();
  }
}
