/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.State;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Base64;
import java.util.Deque;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import static org.junit.Assert.*;

public class BehaviorTreeExecutionStateTest {
  private static BehaviorTreeExecution frame() {
    StateMachine state = new StateMachine(State.createRoot());
    RootContext root = new RootContext(new ActionConstraints(), state);
    Goal goal = Goal.createGoal(Factory.createPredicate("queueTest(self:agent)"));
    GoalContext context = new GoalContext(root, state, goal, ExecutionType.ACT);
    goal.setRootContext(context);
    return new BehaviorTreeExecution(context, "{}");
  }

  private static String encoded(String value) {
    return Base64.getEncoder().encodeToString(value.getBytes(StandardCharsets.UTF_8));
  }

  private static IllegalArgumentException rejects(Runnable operation) {
    try {
      operation.run();
    } catch (IllegalArgumentException expected) {
      return expected;
    }
    throw new AssertionError("Expected an invalid BT value or converter to be rejected");
  }

  @Test
  public void sharedQueuesRejectConvertibleButIncorrectElementTypesWithoutMutatingTheProducer() {
    BehaviorTreeExecution frame = frame();
    Deque<String> queue = new ArrayDeque<>(List.of("1", "2"));
    frame.write("queue", queue);
    IllegalArgumentException error = rejects(() -> frame.value("{queue}", "Queue<int>"));
    assertTrue(error.getMessage(), error.getMessage().contains("java.lang.Integer"));
    assertSame(queue, frame.scriptRead("queue"));
    assertEquals(List.of("1", "2"), new ArrayList<>(queue));
    assertSame(queue, frame.value("{queue}", "Queue<String>"));
    assertEquals(List.of(1, 2), new ArrayList<>((Deque<?>) frame.value("1;2", "Queue<int>")));
  }

  @Test
  public void allSharedQueueTypesKeepTheirIdentityAndRejectMismatches() {
    BehaviorTreeExecution frame = frame();
    for (var entry : Map.of("int", 1, "long", 1L, "double", 1.0,
        "boolean", true, "String", "value").entrySet()) {
      Deque<Object> queue = new ArrayDeque<>(List.of(entry.getValue()));
      frame.write("queue", queue);
      assertSame(queue, frame.value("{queue}", "Queue<" + entry.getKey() + ">"));
      queue.add(new Object());
      rejects(() -> frame.value("{queue}", "Queue<" + entry.getKey() + ">"));
      assertEquals(2, queue.size());
    }
  }

  @Test
  public void loopIntPublicationFailurePreservesTheFirstSharedItemForRetry() {
    BehaviorTreeExecution frame = frame();
    frame.configure(encoded(new BehaviorTreeBindings()
        .bindBelief("flag", "queueTestFlag(self:agent)", "boolean").configuration()));
    Deque<Integer> queue = new ArrayDeque<>(List.of(2, 3));
    frame.write("queue", queue);
    IllegalArgumentException error = rejects(
        () -> frame.queuePop("loop", encoded("{queue}"), encoded("{flag}"), "int"));
    assertTrue(error.getMessage(), error.getMessage().contains("boolean"));
    assertEquals(List.of(2, 3), new ArrayList<>(queue));
    assertTrue(frame.queuePop("loop", encoded("{queue}"), encoded("{local}"), "int"));
    assertEquals(2, frame.scriptRead("local"));
    assertEquals(List.of(3), new ArrayList<>(queue));
  }

  @Test
  public void customConvertersCannotShadowAnyBuiltinAliasOrCollectionType() {
    Map<Class<?>, List<String>> aliases = Map.of(
        String.class, List.of("String", "java.lang.String"),
        Symbol.class, List.of("Symbol", "edu.tufts.hrilab.fol.Symbol"),
        Predicate.class, List.of("Predicate", "edu.tufts.hrilab.fol.Predicate"),
        Boolean.class, List.of("boolean", "Boolean", "java.lang.Boolean"),
        Integer.class, List.of("int", "Integer", "java.lang.Integer"),
        Long.class, List.of("long", "Long", "java.lang.Long"),
        Double.class, List.of("double", "Double", "java.lang.Double"));
    aliases.forEach((type, names) -> names.forEach(name -> {
      rejects(() -> BehaviorTreeExecution.registerPortConverter(name, Object.class, text -> new Object()));
      assertEquals(type, BehaviorTreeExecution.portType(name));
    }));
    for (String collection : List.of("List", "Queue")) {
      for (String element : List.of("int", "long", "double", "boolean", "String")) {
        String name = collection + "<" + element + ">";
        rejects(() -> BehaviorTreeExecution.registerPortConverter(name, Object.class, text -> new Object()));
        assertEquals(collection.equals("List") ? List.class : Deque.class, BehaviorTreeExecution.portType(name));
      }
    }
    for (String name : List.of("java.util.List", "java.util.Queue", "java.util.Deque")) {
      rejects(() -> BehaviorTreeExecution.registerPortConverter(name, Object.class, text -> new Object()));
    }
  }

  @Test
  public void convertersRequireUniqueDottedNamesAndRemainSharedAcrossInvocations() {
    for (String name : List.of("Point", "my..Point", ".Point", "my.Point.", "my.bad-name.Point")) {
      rejects(() -> BehaviorTreeExecution.registerPortConverter(name, Integer.class, Integer::valueOf));
    }
    String name = "diarc.tests.Number" + UUID.randomUUID().toString().replace("-", "");
    BehaviorTreeExecution.registerPortConverter(name, Integer.class, Integer::valueOf);
    assertEquals(Integer.class, BehaviorTreeExecution.portType(name));
    assertEquals(42, frame().value("42", name));
    assertEquals(43, frame().value("43", name));
    rejects(() -> BehaviorTreeExecution.registerPortConverter(name, String.class, text -> text));
    assertEquals(Integer.class, BehaviorTreeExecution.portType(name));
  }
}
