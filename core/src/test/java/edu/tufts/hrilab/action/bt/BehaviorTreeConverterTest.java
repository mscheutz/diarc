/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import edu.tufts.hrilab.action.db.ActionDBEntry;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.nio.file.Files;
import java.util.List;

import static org.junit.Assert.*;

public class BehaviorTreeConverterTest {
  @Rule public TemporaryFolder temporary = new TemporaryFolder();

  static BehaviorTreeBindings bindings() {
    return new BehaviorTreeBindings()
        .bind("Record", new BehaviorTreeBindings.Leaf("btRecord", true, "action",
            new BehaviorTreeBindings.Port("label", "Symbol")))
        .bind("Check", new BehaviorTreeBindings.Leaf("btCheck", true, "boolean",
            new BehaviorTreeBindings.Port("label", "Symbol")))
        .bind("Work", new BehaviorTreeBindings.Leaf("btWork", false, "action",
            new BehaviorTreeBindings.Port("label", "Symbol")));
  }

  static String xml(String body) {
    return "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">" + body + "</BehaviorTree></root>";
  }

  private static IllegalArgumentException rejects(java.util.concurrent.Callable<?> conversion) throws Exception {
    try {
      conversion.call();
    } catch (IllegalArgumentException expected) {
      return expected;
    }
    throw new AssertionError("Expected conversion to reject unsupported or malformed input");
  }

  @Test
  public void bindingRegistrationRejectsAllNativeKindsAndWrapperNames() throws Exception {
    for (String name : List.of("Sequence", "SequenceWithMemory", "Fallback", "ReactiveSequence",
        "ReactiveFallback", "Parallel", "ParallelAll", "IfThenElse", "WhileDoElse", "AsyncSequence",
        "AsyncFallback", "TryCatch", "Switch2", "Switch3", "Switch4", "Switch5", "Switch6",
        "Inverter", "ForceSuccess", "ForceFailure", "RetryUntilSuccessful", "Repeat", "Timeout", "Delay",
        "KeepRunningUntilFailure", "RunOnce", "Precondition", "LoopInt", "LoopBool", "LoopDouble",
        "LoopString", "SkipUnlessUpdated", "WaitValueUpdate", "AlwaysSuccess", "AlwaysFailure",
        "Script", "ScriptCondition", "SetBlackboard", "UnsetBlackboard", "Sleep", "WasEntryUpdated",
        "SubTree", "Action", "Condition", "Control", "Decorator")) {
      IllegalArgumentException error = rejects(
          () -> new BehaviorTreeBindings().bind(name, new BehaviorTreeBindings.Leaf("nativeLeaf", true, "action")));
      assertTrue(error.getMessage(), error.getMessage().contains("reserved"));
      assertTrue(error.getMessage(), error.getMessage().contains(name));
    }
    BehaviorTreeBindings bindings = new BehaviorTreeBindings()
        .bind("CustomSequence", new BehaviorTreeBindings.Leaf("nativeLeaf", true, "action"));
    assertTrue(new BehaviorTreeConverter(bindings).convert(xml("<CustomSequence/>"), "tree")
        .contains("act:nativeLeaf"));
  }

  @Test
  public void generatesLoadableNativeAsl() {
    String asl = new BehaviorTreeConverter(bindings()).convert(xml("""
        <ReactiveSequence><Check label="battery"/><Work label="move"/></ReactiveSequence>
        """), "mission");
    List<ActionDBEntry> actions = BehaviorTreeConverter.parse(asl);
    assertTrue(actions.stream().anyMatch(entry -> entry.getName().equals("mission_bt_2_execute")));
    assertEquals("mission", actions.get(0).getName());
    assertTrue(asl.contains("async {"));
    assertTrue(asl.contains("act:btWork"));
    assertFalse(asl.contains("executeTree"));
  }

  @Test
  public void allSupportedControlsParse() {
    for (String control : List.of("Sequence", "SequenceWithMemory", "Fallback",
        "ReactiveSequence", "ReactiveFallback", "Parallel")) {
      assertNotNull(new BehaviorTreeConverter(bindings()).convert(xml("<" + control
          + "><AlwaysSuccess/><AlwaysFailure/></" + control + ">"), "tree"));
    }
    for (String decorator : List.of("Inverter", "ForceSuccess", "ForceFailure",
        "KeepRunningUntilFailure", "RetryUntilSuccessful num_attempts=\"3\"",
        "Repeat num_cycles=\"2\"", "Timeout msec=\"30\"", "Delay delay_msec=\"20\"")) {
      String name = decorator.split(" ")[0];
      assertNotNull(new BehaviorTreeConverter(bindings()).convert(xml("<" + decorator
          + "><AlwaysSuccess/></" + name + ">"), "tree"));
    }
  }

  @Test
  public void rejectsUnsupportedOrMalformedInput() throws Exception {
    BehaviorTreeConverter converter = new BehaviorTreeConverter(bindings());
    for (String body : List.of("<Unknown/>", "<Sequence/>", "<Inverter><AlwaysSuccess/><AlwaysSuccess/></Inverter>",
        "<Record/>", "<Record label=\"ok\" typo=\"bad\"/>",
        "<Sequence _unknownHook=\"true\"><AlwaysSuccess/></Sequence>",
        "<RetryUntilSuccessful num_attempts=\"oops\"><AlwaysFailure/></RetryUntilSuccessful>",
        "<SubTree ID=\"Missing\"/>")) {
      rejects(() -> converter.convert(xml(body), "tree"));
    }
    rejects(() -> converter.convert(xml("<AlwaysSuccess/>"), "bad-name"));
    rejects(() -> converter.convert("<root><BehaviorTree ID=\"Main\"><AlwaysSuccess/></BehaviorTree></root>", "tree"));
  }

  @Test
  public void rejectsExternalEntitiesAndCompilesRecursiveSubtreeCalls() throws Exception {
    rejects(() -> new BehaviorTreeConverter(bindings()).convert(
        "<!DOCTYPE root [<!ENTITY file SYSTEM 'file:///etc/passwd'>]>"
            + xml("<Record label=\"&file;\"/>"), "tree"));
    String recursive = new BehaviorTreeConverter(bindings()).convert(xml("<SubTree ID=\"Main\"/>"), "tree");
    assertTrue(recursive.contains("\"subtree\""));
    assertTrue(recursive.length() < 20000);
  }

  @Test
  public void includesAreRelativeAndBounded() throws Exception {
    var directory = temporary.newFolder().toPath();
    Files.writeString(directory.resolve("child.xml"), xml("<Record label=\"included\"/>")
        .replace("ID=\"Main\"", "ID=\"Child\""));
    var main = directory.resolve("main.xml");
    Files.writeString(main, """
        <root BTCPP_format="4" main_tree_to_execute="Main">
          <include path="child.xml"/>
          <BehaviorTree ID="Main"><SubTree ID="Child"/></BehaviorTree>
        </root>
        """);
    assertTrue(new BehaviorTreeConverter(bindings()).convert(main, "tree").contains("act:btRecord"));
    var outside = temporary.newFile("outside.xml").toPath();
    Files.writeString(outside, xml("<AlwaysSuccess/>"));
    Files.writeString(main, "<root BTCPP_format=\"4\"><include path=\"../outside.xml\"/></root>");
    rejects(() -> new BehaviorTreeConverter(bindings()).convert(main, "tree"));
  }
}
