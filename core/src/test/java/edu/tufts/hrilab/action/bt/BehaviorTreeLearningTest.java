/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.learning.ActionLearning;
import edu.tufts.hrilab.action.learning.ActionLearningStatus;
import edu.tufts.hrilab.action.learning.BehaviorTreeLearning;
import edu.tufts.hrilab.action.learning.LearningState;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Path;
import java.io.StringReader;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadInfo;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.Set;
import java.util.concurrent.FutureTask;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

import static org.junit.Assert.*;

public class BehaviorTreeLearningTest {
  private static final String BINDINGS_FILE = "src/test/resources/bt/learning-bindings.json";
  private static final Symbol ACTOR = Factory.createSymbol("self:agent");
  private static final AtomicInteger NAMES = new AtomicInteger();
  private static GoalManagerComponent manager;
  private final List<String> trace = new CopyOnWriteArrayList<>();
  private final List<Long> submitted = new ArrayList<>();
  private Collection<TRADEServiceInfo> services;
  private BehaviorTreeLearning learning;
  private String name;

  @BeforeClass
  public static void initialize() {
    manager = DiarcComponent.createInstance(GoalManagerComponent.class, true, false,
        "-beliefinitfile agents/agents.pl -nomemorymanager");
  }

  @AfterClass
  public static void shutdown() {
    manager.shutdownComponent();
  }

  @Before
  public void setup() throws Exception {
    services = TRADE.registerAllServices(this, List.of());
    Database.getInstance().addPrimitiveActions(null);
    learning = new ActionLearning(null, false).getBehaviorTreeLearning();
    name = "btLearnTree" + NAMES.incrementAndGet();
    learning.beginFromFile(name, ACTOR, "Sequence", BINDINGS_FILE);
  }

  @After
  public void cleanup() throws Exception {
    for (long id : submitted) {
      if (!manager.getGoalStatus(id).isTerminated()) {
        manager.cancelGoal(id);
      }
      assertTrue(manager.joinOnGoal(id, 5000).isTerminated());
    }
    for (var entry : Database.getActionDB().getAllActions()) {
      if (services.contains(entry.getServiceInfo())) {
        Database.getInstance().removeActionDBEntry(entry);
      }
    }
    TRADE.deregister(this);
  }

  @TRADEService
  @Action
  public void btLearnRecord(Symbol label) {
    trace.add(label.getName());
  }

  @TRADEService
  @Action
  public boolean btLearnCheck(Boolean value) {
    return value;
  }

  private void record(String parent, String id, String label, int index) {
    learning.record(name, parent, id, "Record",
        Factory.createPredicate("btLearnRecord", ACTOR, Factory.createSymbol(label)), index);
  }

  private void execute(String action) {
    long id = manager.submitGoal(Factory.createPredicate(action, ACTOR));
    submitted.add(id);
    assertEquals("Trace: " + trace, GoalStatus.SUCCEEDED, manager.joinOnGoal(id, 5000));
  }

  @Test(timeout = 20000)
  public void nestedStructureExportsReloadsAndExecutesNativeActions() throws Exception {
    learning.addControl(name, "root", "choice", "Fallback", 0);
    learning.addControl(name, "choice", "preferred", "Sequence", 0);
    learning.record(name, "preferred", "check", "Check",
        Factory.createPredicate("btLearnCheck", ACTOR, Factory.createSymbol("false")), 0);
    record("preferred", "skipped", "skipped", 1);
    record("choice", "recovery", "recovery", 1);
    record("root", "done", "done", 1);

    String xml = learning.getXml(name);
    assertTrue(xml.contains("<Fallback name=\"choice\">"));
    assertTrue(xml.contains("<Sequence name=\"preferred\">"));
    assertTrue(xml.contains("<Action ID=\"Record\" name=\"recovery\" label=\"recovery\"/>"));
    assertTrue(learning.finish(name));
    execute(name);
    assertEquals(List.of("recovery", "done"), trace);
    assertEquals(xml, learning.getXml(name));

    Path directory = Path.of("build", "bt-learning-" + name);
    Files.createDirectories(directory);
    Path xmlFile = directory.resolve("tree.xml");
    Path bindingsFile = directory.resolve("bindings.json");
    try {
      learning.export(name, xmlFile.toString(), bindingsFile.toString());
      assertEquals(xml, Files.readString(xmlFile));
      String reloaded = name + "Reloaded";
      manager.loadBehaviorTree(xmlFile.toString(), bindingsFile.toString(), reloaded);
      execute(reloaded);
      assertEquals(List.of("recovery", "done", "recovery", "done"), trace);
    } finally {
      Files.deleteIfExists(xmlFile);
      Files.deleteIfExists(bindingsFile);
      Files.deleteIfExists(directory);
    }
  }

  @Test(timeout = 15000)
  public void actionLearningRetainsLinearBehaviorAndLearnedAslIsALeaf() {
    ActionLearning legacy = new ActionLearning(null, false);
    assertSame(legacy.getBehaviorTreeLearning(), legacy.getBehaviorTreeLearning());
    assertEquals(ActionLearningStatus.NONE, legacy.getLearningStatus());
    assertTrue(legacy.shouldIgnore(Goal.createGoal(Factory.createPredicate(
        "addBehaviorTreeControl(self:agent,tree,root,choice,Fallback,0)"))));
    assertTrue(legacy.shouldIgnore(Goal.createGoal(Factory.createPredicate(
        "addBehaviorTreeDefinition(self:agent,tree,Helper,helperRoot,Sequence,attributes)"))));
    assertTrue(legacy.shouldIgnore(Goal.createGoal(Factory.createPredicate(
        "addBehaviorTreeSubtree(self:agent,tree,root,call,Helper,remaps,0)"))));
    LearningState linear = new LearningState(Factory.createPredicate("btLearnedLinear", ACTOR));
    linear.createEventSpec(Goal.createGoal(Factory.createPredicate("btLearnRecord", ACTOR,
        Factory.createSymbol("linear"))));
    assertEquals(1, linear.getEventSpecs().size());
    linear.generateActionDBEntry();

    learning.record(name, "root", "linear", "LearnedLinear",
        Factory.createPredicate("btLearnedLinear", ACTOR), 0);
    learning.finish(name);
    execute(name);
    assertEquals(List.of("linear"), trace);
    assertEquals(ActionLearningStatus.NONE, legacy.getLearningStatus());
  }

  @Test
  public void editsAreOrderedAndRejectInvalidChangesWithoutDroppingSteps() {
    learning.addControl(name, "root", "nested", "Fallback", 0);
    record("nested", "one", "one", 0);
    record("root", "two", "two", 1);
    String original = learning.getXml(name);
    reject(() -> learning.addControl(name, "root", "nested", "Sequence", 0));
    reject(() -> learning.addControl(name, "root", "bad", "ImaginaryControl", 0));
    reject(() -> record("one", "childOfLeaf", "bad", 0));
    reject(() -> record("root", "badIndex", "bad", 99));
    reject(() -> learning.move(name, "nested", "one", 0));
    reject(() -> learning.remove(name, "root"));
    reject(() -> learning.remove(name, "missing"));
    reject(() -> learning.addLeaf(name, "root", "unknown", "Unbound", "{}", 0));
    reject(() -> learning.addLeaf(name, "root", "missingPort", "Record", "{}", 0));
    reject(() -> learning.addLeaf(name, "root", "extraPort", "Record",
        "{\"label\":\"ok\",\"bogus\":\"not ignored\"}", 0));
    reject(() -> learning.record(name, "root", "mismatch", "Record",
        Factory.createPredicate("different(self:agent,x)"), 0));
    reject(() -> learning.record(name, "root", "otherActor", "Record",
        Factory.createPredicate("btLearnRecord(other:agent,x)"), 0));
    reject(() -> learning.record(name, "root", "goalInstruction", "Record",
        Factory.createPredicate("goal(self:agent,notAnActionCall)"), 0));
    assertEquals(original, learning.getXml(name));
    learning.move(name, "two", "nested", 0);
    assertTrue(learning.getXml(name).indexOf("name=\"two\"") < learning.getXml(name).indexOf("name=\"one\""));
    learning.remove(name, "one");
    assertFalse(learning.getXml(name).contains("name=\"one\""));
    learning.remove(name, "nested");
    reject(() -> learning.getXml(name));
    record("root", "repair", "repaired", 0);
    assertTrue(learning.getXml(name).contains("repaired"));
  }

  @Test
  public void explicitPortsRetainRemappingAndXmlEscaping() {
    learning.addLeaf(name, "root", "mapped", "Record", "{\"label\":\"{target}\"}", 0);
    learning.addLeaf(name, "root", "escaped", "Record", "{\"label\":\"a & b < \\\"c\\\"\"}", 1);
    String xml = learning.getXml(name);
    assertTrue(xml.contains("label=\"{target}\""));
    assertTrue(xml.contains("a &amp; b &lt; &quot;c&quot;"));
    String outputs = name + "Outputs";
    learning.begin(outputs, ACTOR, "Sequence",
        "{\"Return\":{\"action\":\"btLearnCheck\",\"synchronous\":true,\"result\":\"boolean\","
            + "\"inputs\":[{\"name\":\"value\",\"type\":\"Boolean\"}],"
            + "\"outputs\":[{\"name\":\"result\",\"type\":\"Boolean\"}]}}");
    reject(() -> learning.record(outputs, "root", "implicitOutput", "Return",
        Factory.createPredicate("btLearnCheck(self:agent,true)"), 0));
    reject(() -> learning.addLeaf(outputs, "root", "badOutput", "Return",
        "{\"value\":\"true\",\"result\":\"literal\"}", 0));
    learning.addLeaf(outputs, "root", "explicitOutput", "Return",
        "{\"value\":\"true\",\"result\":\"{checkResult}\"}", 0);
    assertTrue(learning.getXml(outputs).contains("result=\"{checkResult}\""));
  }

  @Test
  public void incompleteFailedPublishAndCancelledDraftsDoNotLoseState() {
    reject(() -> learning.finish(name));
    assertNull(Database.getActionDB().getAction(name));
    record("root", "record", "present", 0);
    learning.finish(name);
    String xml = learning.getXml(name);
    reject(() -> learning.remove(name, "record"));
    reject(() -> learning.cancel(name));
    assertEquals(xml, learning.getXml(name));
    String cancelled = name + "Cancelled";
    learning.begin(cancelled, ACTOR, "Fallback", learning.getBindingsJson(name));
    assertTrue(learning.cancel(cancelled));
    reject(() -> learning.getXml(cancelled));
  }

  @Test
  public void actorTypesDoNotChangeRecordingAndReservedPortsCannotReplaceNodeMetadata() {
    learning.record(name, "root", "untypedActor", "Record",
        Factory.createPredicate("btLearnRecord(self,untyped)"), 0);
    assertTrue(learning.getXml(name).contains("label=\"untyped\""));
    String reserved = name + "Reserved";
    learning.begin(reserved, ACTOR, "Sequence",
        "{\"Reserved\":{\"action\":\"btLearnRecord\",\"synchronous\":true,"
            + "\"inputs\":[{\"name\":\"name\",\"type\":\"Symbol\",\"defaultValue\":\"original\"}]}}");
    reject(() -> learning.addLeaf(reserved, "root", "wouldOverwriteDefault", "Reserved", "{}", 0));
    reject(() -> learning.addLeaf(reserved, "root", "wouldDuplicateAttribute", "Reserved",
        "{\"name\":\"explicit\"}", 0));
  }

  @Test
  public void conflictingNativeActionNamesLeaveDraftsEditableAndInstallNoHelpers() {
    new edu.tufts.hrilab.action.db.ActionDBEntry.Builder(name).build(true);
    record("root", "first", "first", 0);
    String before = learning.getXml(name);
    reject(() -> learning.finish(name));
    assertNull(Database.getActionDB().getAction(name + "_bt_0"));
    assertEquals(before, learning.getXml(name));
    record("root", "second", "second", 1);
    assertTrue(learning.getXml(name).contains("name=\"second\""));
  }

  @Test
  public void genericNodesCoverAllNativeControlsDecoratorsAndBuiltinLeaves() {
    Map<String, String> parameters = new LinkedHashMap<>();
    for (String kind : List.of("Sequence", "SequenceWithMemory", "Fallback", "ReactiveSequence",
        "ReactiveFallback", "AsyncSequence", "AsyncFallback", "IfThenElse", "WhileDoElse",
        "TryCatch", "Inverter", "ForceSuccess", "ForceFailure", "KeepRunningUntilFailure", "RunOnce")) {
      parameters.put(kind, "{}");
    }
    parameters.put("Parallel", "{\"success_count\":\"1\",\"failure_count\":\"1\"}");
    parameters.put("ParallelAll", "{\"max_failures\":\"1\"}");
    parameters.put("RetryUntilSuccessful", "{\"num_attempts\":\"2\"}");
    parameters.put("Repeat", "{\"num_cycles\":\"2\"}");
    parameters.put("Timeout", "{\"msec\":\"10\"}");
    parameters.put("Delay", "{\"delay_msec\":\"1\"}");
    parameters.put("Precondition", "{\"if\":\"true\",\"else\":\"FAILURE\"}");
    for (String kind : List.of("LoopInt", "LoopBool", "LoopDouble", "LoopString")) {
      parameters.put(kind, "{\"queue\":\"{queue}\",\"value\":\"{item}\",\"if_empty\":\"SUCCESS\"}");
    }
    parameters.put("SkipUnlessUpdated", "{\"entry\":\"{target}\"}");
    parameters.put("WaitValueUpdate", "{\"entry\":\"{target}\"}");
    parameters.put("AlwaysSuccess", "{}");
    parameters.put("AlwaysFailure", "{}");
    parameters.put("Script", "{\"code\":\"local := 1\"}");
    parameters.put("ScriptCondition", "{\"code\":\"true\"}");
    parameters.put("SetBlackboard", "{\"value\":\"ready\",\"output_key\":\"target\"}");
    parameters.put("UnsetBlackboard", "{\"key\":\"target\"}");
    parameters.put("Sleep", "{\"msec\":\"1\"}");
    parameters.put("WasEntryUpdated", "{\"entry\":\"{target}\"}");
    for (int cases = 2; cases <= 6; cases++) {
      Map<String, String> ports = new LinkedHashMap<>();
      ports.put("variable", "{choice}");
      for (int c = 1; c <= cases; c++) {
        ports.put("case_" + c, "value" + c);
      }
      parameters.put("Switch" + cases, new com.google.gson.Gson().toJson(ports));
    }
    List<String> leaves = List.of("AlwaysSuccess", "AlwaysFailure", "Script", "ScriptCondition",
        "SetBlackboard", "UnsetBlackboard", "Sleep", "WasEntryUpdated");
    for (Map.Entry<String, String> entry : parameters.entrySet()) {
      String kind = entry.getKey();
      String tree = name + kind;
      learning.begin(tree, ACTOR, "Sequence", "{}");
      learning.addNode(tree, "root", "native", kind, entry.getValue(), 0);
      int arity = leaves.contains(kind) ? 0
          : kind.startsWith("Switch") ? Integer.parseInt(kind.substring(6)) + 1
          : List.of("IfThenElse", "WhileDoElse", "TryCatch").contains(kind) ? 2 : 1;
      for (int child = 0; child < arity; child++) {
        learning.addNode(tree, "native", "child" + child, "AlwaysSuccess", "{}", child);
      }
      assertTrue(kind, learning.getXml(tree).contains("<" + kind + " name=\"native\""));
    }
  }

  @Test(timeout = 15000)
  public void parameterizedDecoratorRootExecutesAndGenericArityValidationIsDeferred() {
    String tree = name + "RootDecorator";
    learning.begin(tree, ACTOR, "Repeat", learning.getBindingsJson(name), "{\"num_cycles\":\"1\"}");
    reject(() -> learning.getXml(tree));
    learning.addNode(tree, "root", "record", "Record", "{\"label\":\"decorated\"}", 0);
    learning.addNode(tree, "root", "extra", "AlwaysSuccess", "{}", 1);
    reject(() -> learning.getXml(tree));
    learning.remove(tree, "extra");
    assertTrue(learning.getXml(tree).contains("<Repeat name=\"root\" num_cycles=\"1\">"));
    learning.finish(tree);
    execute(tree);
    assertEquals(List.of("decorated"), trace);

    learning.addNode(name, "root", "invalidAttributes", "AlwaysSuccess", "{\"unsupported\":\"retained\"}", 0);
    reject(() -> learning.getXml(name));
    learning.remove(name, "invalidAttributes");
    learning.addNode(name, "root", "valid", "AlwaysSuccess", "{}", 0);
    assertTrue(learning.getXml(name).contains("name=\"valid\""));
    reject(() -> learning.addNode(name, "root", "unknown", "ImaginaryBuiltin", "{}", 0));
  }

  @Test(timeout = 20000)
  public void goalManagerServicesTeachPublishExportAndExecuteTheirActorOwnedModel() throws Exception {
    String tree = name + "Managed";
    long begin = manager.submitGoal(Factory.createPredicate("beginBehaviorTreeLearning", ACTOR,
        new Symbol(tree), new Symbol("Sequence"), new Symbol(BINDINGS_FILE)));
    submitted.add(begin);
    assertEquals(GoalStatus.SUCCEEDED, manager.joinOnGoal(begin, 5000));
    assertTrue(manager.addBehaviorTreeControl(ACTOR, tree, "root", "choice", "Fallback", 0));
    assertTrue(manager.addBehaviorTreeNode(ACTOR, tree, "choice", "failed", "AlwaysFailure", "{}", 0));
    assertTrue(manager.addBehaviorTreeLeaf(ACTOR, tree, "choice", "recovered", "Record",
        "{\"label\":\"managerRecovery\"}", 1));
    assertTrue(manager.recordBehaviorTreeStep(ACTOR, tree, "root", "done", "Record",
        Factory.createPredicate("btLearnRecord(self:agent,managerDone)"), 1));
    assertTrue(manager.moveBehaviorTreeNode(ACTOR, tree, "done", "choice", 2));
    assertTrue(manager.moveBehaviorTreeNode(ACTOR, tree, "done", "root", 1));
    assertTrue(manager.addBehaviorTreeNode(ACTOR, tree, "root", "removed", "AlwaysFailure", "{}", 2));
    assertTrue(manager.removeBehaviorTreeNode(ACTOR, tree, "removed"));
    String xml = TRADE.getAvailableService(new TRADEServiceConstraints()
        .name("getLearnedBehaviorTreeXml").argTypes(Symbol.class, String.class))
        .call(String.class, ACTOR, tree);
    assertTrue(xml.contains("<Fallback name=\"choice\">"));
    assertFalse(xml.contains("name=\"removed\""));
    assertTrue(TRADE.getAvailableService(new TRADEServiceConstraints()
        .name("endBehaviorTreeLearning").argTypes(Symbol.class, String.class))
        .call(Boolean.class, ACTOR, tree));
    execute(tree);
    assertEquals(List.of("managerRecovery", "managerDone"), trace);
    assertEquals(xml, manager.getLearnedBehaviorTreeXml(ACTOR, tree));

    String cancelled = tree + "Cancelled";
    assertTrue(TRADE.getAvailableService(new TRADEServiceConstraints()
        .name("beginBehaviorTreeLearning")
        .argTypes(Symbol.class, String.class, String.class, String.class, String.class))
        .call(Boolean.class, ACTOR, cancelled, "Repeat", BINDINGS_FILE, "{\"num_cycles\":\"1\"}"));
    assertTrue(manager.addBehaviorTreeNode(ACTOR, cancelled, "root", "child", "AlwaysSuccess", "{}", 0));
    assertTrue(manager.getLearnedBehaviorTreeXml(ACTOR, cancelled).contains("num_cycles=\"1\""));
    assertTrue(manager.cancelBehaviorTreeLearning(ACTOR, cancelled));
    reject(() -> manager.getLearnedBehaviorTreeXml(ACTOR, cancelled));
    reject(() -> manager.getLearnedBehaviorTreeXml(Factory.createSymbol("unknownLearningActor"), tree));
    reject(() -> manager.getLearnedBehaviorTreeXml(null, tree));

    Path directory = Path.of("build", "bt-learning-" + tree);
    Files.createDirectories(directory);
    Path xmlFile = directory.resolve("tree.xml");
    Path bindingsFile = directory.resolve("bindings.json");
    try {
      assertTrue(manager.exportBehaviorTree(ACTOR, tree, xmlFile.toString(), bindingsFile.toString()));
      assertEquals(xml, Files.readString(xmlFile));
      manager.loadBehaviorTree(xmlFile.toString(), bindingsFile.toString(), tree + "Reloaded");
      execute(tree + "Reloaded");
      assertEquals(List.of("managerRecovery", "managerDone", "managerRecovery", "managerDone"), trace);
    } finally {
      Files.deleteIfExists(xmlFile);
      Files.deleteIfExists(bindingsFile);
      Files.deleteIfExists(directory);
    }
  }

  private void teachRecursiveDefinition(String tree, String definition, String target) {
    String root = definition + "Root";
    assertTrue(manager.addBehaviorTreeDefinition(ACTOR, tree, definition, root, "IfThenElse", "{}"));
    manager.addBehaviorTreeNode(ACTOR, tree, root, definition + "Test", "ScriptCondition",
        "{\"code\":\"depth == 0\"}", 0);
    manager.addBehaviorTreeLeaf(ACTOR, tree, root, definition + "Base", "Record",
        "{\"label\":\"" + definition + "Base\"}", 1);
    String body = definition + "Body";
    manager.addBehaviorTreeControl(ACTOR, tree, root, body, "Sequence", 2);
    manager.addBehaviorTreeLeaf(ACTOR, tree, body, definition + "Enter", "Record",
        "{\"label\":\"" + definition + "Enter\"}", 0);
    manager.addBehaviorTreeNode(ACTOR, tree, body, definition + "Next", "Script",
        "{\"code\":\"next := depth - 1\"}", 1);
    assertTrue(manager.addBehaviorTreeSubtree(ACTOR, tree, body, definition + "Call", target,
        "{\"depth\":\"{next}\"}", 2));
    manager.addBehaviorTreeLeaf(ACTOR, tree, body, definition + "Leave", "Record",
        "{\"label\":\"" + definition + "Leave\"}", 3);
  }

  private void teachRecursiveMain(String tree, String target) throws Exception {
    manager.beginBehaviorTreeLearning(ACTOR, tree, "Sequence", BINDINGS_FILE);
    manager.addBehaviorTreeNode(ACTOR, tree, "root", "initialize", "Script", "{\"code\":\"n := 3\"}", 0);
    manager.addBehaviorTreeSubtree(ACTOR, tree, "root", "initialCall", target, "{\"depth\":\"{n}\"}", 1);
    manager.addBehaviorTreeNode(ACTOR, tree, "root", "callerUnchanged", "ScriptCondition",
        "{\"code\":\"n == 3\"}", 2);
  }

  private void publishExportReloadRecursive(String tree, List<String> expected) throws Exception {
    String xml = manager.getLearnedBehaviorTreeXml(ACTOR, tree);
    manager.endBehaviorTreeLearning(ACTOR, tree);
    execute(tree);
    assertEquals(expected, trace);
    Path directory = Path.of("build", "bt-learning-" + tree);
    Files.createDirectories(directory);
    Path xmlFile = directory.resolve("tree.xml");
    Path bindingsFile = directory.resolve("bindings.json");
    try {
      manager.exportBehaviorTree(ACTOR, tree, xmlFile.toString(), bindingsFile.toString());
      assertEquals(xml, Files.readString(xmlFile));
      manager.loadBehaviorTree(xmlFile.toString(), bindingsFile.toString(), tree + "Reloaded");
      trace.clear();
      execute(tree + "Reloaded");
      assertEquals(expected, trace);
    } finally {
      Files.deleteIfExists(xmlFile);
      Files.deleteIfExists(bindingsFile);
      Files.deleteIfExists(directory);
    }
  }

  @Test(timeout = 20000)
  public void directRecursiveDefinitionsRetainCallsAndExportReloadExecute() throws Exception {
    String tree = name + "Direct";
    teachRecursiveMain(tree, "Depth");
    teachRecursiveDefinition(tree, "Depth", "Depth");
    String xml = manager.getLearnedBehaviorTreeXml(ACTOR, tree);
    assertTrue(xml.contains("<BehaviorTree ID=\"Depth\">"));
    assertTrue(xml.contains("<SubTree ID=\"Depth\" name=\"DepthCall\" depth=\"{next}\"/>"));
    publishExportReloadRecursive(tree,
        List.of("DepthEnter", "DepthEnter", "DepthEnter", "DepthBase",
            "DepthLeave", "DepthLeave", "DepthLeave"));
  }

  @Test(timeout = 20000)
  public void mutualRecursiveDefinitionsRetainForwardCallsAndExportReloadExecute() throws Exception {
    String tree = name + "Mutual";
    teachRecursiveMain(tree, "Even");
    teachRecursiveDefinition(tree, "Even", "Odd");
    teachRecursiveDefinition(tree, "Odd", "Even");
    String xml = manager.getLearnedBehaviorTreeXml(ACTOR, tree);
    assertTrue(xml.contains("<BehaviorTree ID=\"Even\">"));
    assertTrue(xml.contains("<BehaviorTree ID=\"Odd\">"));
    assertTrue(xml.contains("<SubTree ID=\"Odd\" name=\"EvenCall\""));
    assertTrue(xml.contains("<SubTree ID=\"Even\" name=\"OddCall\""));
    publishExportReloadRecursive(tree,
        List.of("EvenEnter", "OddEnter", "EvenEnter", "OddBase", "EvenLeave", "OddLeave", "EvenLeave"));
  }

  @Test
  public void definitionEditsRejectDuplicateIdsAndStructuralCyclesButPermitCallGraphCycles() {
    record("root", "mainStep", "main", 0);
    learning.addDefinition(name, "Helper", "helperRoot", "Sequence", "{}");
    learning.addControl(name, "helperRoot", "outer", "Sequence", 0);
    learning.addControl(name, "outer", "inner", "Sequence", 0);
    learning.addNode(name, "inner", "helperLeaf", "AlwaysSuccess", "{}", 0);
    String before = learning.getXml(name);
    reject(() -> learning.addDefinition(name, "Helper", "anotherRoot", "Sequence", "{}"));
    reject(() -> learning.addDefinition(name, name, "anotherRoot", "Sequence", "{}"));
    reject(() -> learning.addDefinition(name, "DuplicateRoot", "root", "Sequence", "{}"));
    reject(() -> learning.addDefinition(name, "DuplicateChild", "mainStep", "Sequence", "{}"));
    reject(() -> learning.addNode(name, "helperRoot", "mainStep", "AlwaysSuccess", "{}", 1));
    reject(() -> learning.move(name, "outer", "inner", 0));
    reject(() -> learning.remove(name, "helperRoot"));
    reject(() -> learning.move(name, "helperRoot", "root", 0));
    assertEquals(before, learning.getXml(name));

    learning.addSubtree(name, "root", "unresolved", "Missing", "{}", 1);
    reject(() -> learning.getXml(name));
    reject(() -> learning.addNode(name, "unresolved", "childOfCall", "AlwaysSuccess", "{}", 0));
    learning.remove(name, "unresolved");
    learning.addSubtree(name, "root", "invalidRemap", "Helper", "{\"_autoremap\":\"notBoolean\"}", 1);
    reject(() -> learning.getXml(name));
    learning.remove(name, "invalidRemap");
    learning.addDefinition(name, "Unused", "unusedRoot", "Sequence", "{}");
    reject(() -> learning.getXml(name));
    learning.addNode(name, "unusedRoot", "unusedChild", "AlwaysSuccess", "{}", 0);
    learning.addSubtree(name, "helperRoot", "recursiveCall", "Helper", "{}", 1);
    assertTrue(learning.getXml(name).contains("<SubTree ID=\"Helper\" name=\"recursiveCall\"/>"));
    learning.move(name, "helperLeaf", "root", 1);
    reject(() -> learning.getXml(name));
    learning.addNode(name, "inner", "repairedInner", "AlwaysSuccess", "{}", 0);
    assertTrue(learning.getXml(name).contains("name=\"helperLeaf\""));
  }

  private BehaviorTreeLearning competingDraft(String label, boolean nested) {
    BehaviorTreeLearning draft = new ActionLearning(null, false).getBehaviorTreeLearning();
    draft.begin(name, Factory.createSymbol(label + ":agent"), "Sequence", learning.getBindingsJson(name));
    String parent = "root";
    if (nested) {
      draft.addControl(name, "root", "nested", "Sequence", 0);
      parent = "nested";
    }
    draft.addLeaf(name, parent, "record", "Record", "{\"label\":\"" + label + "\"}", 0);
    return draft;
  }

  @FunctionalInterface
  private interface Publication {
    void publish() throws Exception;
  }

  private static FutureTask<Boolean> publicationTask(Publication publication) {
    return new FutureTask<>(() -> {
      try {
        publication.publish();
        return true;
      } catch (IllegalArgumentException conflict) {
        assertTrue(conflict.getMessage(), conflict.getMessage().contains("already exists"));
        return false;
      }
    });
  }

  private boolean[] racePublications(Publication first, Publication second) throws Exception {
    FutureTask<Boolean> firstTask = publicationTask(first);
    FutureTask<Boolean> secondTask = publicationTask(second);
    Thread firstThread = new Thread(firstTask, "bt-learning-publication-one");
    Thread secondThread = new Thread(secondTask, "bt-learning-publication-two");
    Object database = Database.getActionDB();
    try {
      synchronized (database) {
        firstThread.start();
        secondThread.start();
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(10);
        while ((!blockedAtPublication(firstThread, database) || !blockedAtPublication(secondThread, database))
            && System.nanoTime() < deadline) {
          Thread.sleep(5);
        }
        assertTrue("First publisher did not wait for the global publication lock",
            blockedAtPublication(firstThread, database));
        assertTrue("Second publisher did not wait for the global publication lock",
            blockedAtPublication(secondThread, database));
        assertNull(Database.getActionDB().getAction(name));
        assertFalse(firstTask.isDone());
        assertFalse(secondTask.isDone());
      }
      boolean firstWon = firstTask.get(10, TimeUnit.SECONDS);
      boolean secondWon = secondTask.get(10, TimeUnit.SECONDS);
      assertTrue("Exactly one competing publication must succeed", firstWon ^ secondWon);
      return new boolean[]{firstWon, secondWon};
    } finally {
      firstThread.join(10000);
      secondThread.join(10000);
      assertFalse("First publisher leaked a thread", firstThread.isAlive());
      assertFalse("Second publisher leaked a thread", secondThread.isAlive());
    }
  }

  private static boolean blockedAtPublication(Thread thread, Object database) {
    ThreadInfo info = ManagementFactory.getThreadMXBean().getThreadInfo(thread.getId(), 1);
    if (info == null || info.getThreadState() != Thread.State.BLOCKED || info.getLockInfo() == null
        || info.getLockInfo().getIdentityHashCode() != System.identityHashCode(database)
        || info.getStackTrace().length == 0) {
      return false;
    }
    String method = info.getStackTrace()[0].getMethodName();
    return method.equals("finish") || method.equals("loadBehaviorTree");
  }

  private void assertInstalledPublication(BehaviorTreeLearning winner, String label) {
    List<edu.tufts.hrilab.action.db.ActionDBEntry> expected = new BehaviorTreeConverter(
        BehaviorTreeBindings.read(new StringReader(winner.getBindingsJson(name))))
        .actions(winner.getXml(name), name);
    Set<String> names = expected.stream().map(entry -> entry.getName()).collect(Collectors.toSet());
    List<edu.tufts.hrilab.action.db.ActionDBEntry> installed = Database.getActionDB().getAllActions().stream()
        .filter(entry -> entry.getName().equals(name) || entry.getName().startsWith(name + "_bt_"))
        .collect(Collectors.toList());
    assertEquals("No helpers from the losing model may be installed", names,
        installed.stream().map(entry -> entry.getName()).collect(Collectors.toSet()));
    assertEquals("No duplicate helper versions may be installed", expected.size(), installed.size());
    execute(name);
    assertEquals(List.of(label), trace);
  }

  @Test(timeout = 30000)
  public void actorOwnedPublishersAtomicallyRejectTheSameGlobalActionName() throws Exception {
    BehaviorTreeLearning first = competingDraft("first", false);
    BehaviorTreeLearning second = competingDraft("second", true);
    boolean[] won = racePublications(() -> first.finish(name), () -> second.finish(name));
    BehaviorTreeLearning winner = won[0] ? first : second;
    BehaviorTreeLearning loser = won[0] ? second : first;
    Map<String, edu.tufts.hrilab.action.db.ActionDBEntry> installed = Database.getActionDB().getAllActions().stream()
        .filter(entry -> entry.getName().equals(name) || entry.getName().startsWith(name + "_bt_"))
        .collect(Collectors.toMap(entry -> entry.getName(), entry -> entry));
    loser.addNode(name, "root", "stillEditable", "AlwaysSuccess", "{}", 1);
    assertTrue(loser.getXml(name).contains("name=\"stillEditable\""));
    reject(() -> loser.finish(name));
    installed.forEach((action, entry) -> assertSame(entry, Database.getActionDB().getAction(action)));
    assertInstalledPublication(winner, won[0] ? "first" : "second");
  }

  @Test(timeout = 30000)
  public void runtimeLoadingAndLearningShareTheSameAtomicPublicationLock() throws Exception {
    BehaviorTreeLearning first = competingDraft("learner", false);
    BehaviorTreeLearning source = competingDraft("loader", true);
    Path directory = Path.of("build", "bt-learning-race-" + name);
    Files.createDirectories(directory);
    Path xmlFile = directory.resolve("tree.xml");
    Path bindingsFile = directory.resolve("bindings.json");
    try {
      source.export(name, xmlFile.toString(), bindingsFile.toString());
      boolean[] won = racePublications(() -> first.finish(name),
          () -> manager.loadBehaviorTree(xmlFile.toString(), bindingsFile.toString(), name));
      assertInstalledPublication(won[0] ? first : source, won[0] ? "learner" : "loader");
      if (!won[0]) {
        first.addNode(name, "root", "stillEditable", "AlwaysSuccess", "{}", 1);
        assertTrue(first.getXml(name).contains("name=\"stillEditable\""));
      }
      source.addNode(name, "root", "sourceStillEditable", "AlwaysSuccess", "{}", 1);
      assertTrue(source.getXml(name).contains("name=\"sourceStillEditable\""));
    } finally {
      Files.deleteIfExists(xmlFile);
      Files.deleteIfExists(bindingsFile);
      Files.deleteIfExists(directory);
    }
  }

  private static void reject(Runnable operation) {
    try {
      operation.run();
      fail("Expected invalid structured edit to be rejected");
    } catch (IllegalArgumentException expected) {
      assertNotNull(expected.getMessage());
    }
  }
}
