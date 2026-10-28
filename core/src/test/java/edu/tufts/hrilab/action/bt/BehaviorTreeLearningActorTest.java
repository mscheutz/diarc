/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import edu.tufts.hrilab.action.learning.ActionLearningStatus;
import edu.tufts.hrilab.action.learning.BehaviorTreeLearning;
import edu.tufts.hrilab.action.manager.AgentTeam;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import static org.junit.Assert.*;

/** Covers the actor registry without starting execution or linear learning threads. */
public class BehaviorTreeLearningActorTest {
  private static final class LearningManager extends ExecutionManager {
    void addActor(Symbol actor) {
      agentTeams.put(actor.toUntyped(), new AgentTeam(actor.toUntyped(), this));
    }
  }

  @Test
  public void actorsOwnSeparateDraftsAndTypedLookupReusesTheExistingTeam() {
    LearningManager manager = new LearningManager();
    Symbol one = Factory.createSymbol("one:agent");
    Symbol two = Factory.createSymbol("two:agent");
    manager.addActor(one);
    manager.addActor(two);
    BehaviorTreeLearning first = manager.getBehaviorTreeLearning(one);
    BehaviorTreeLearning second = manager.getBehaviorTreeLearning(two);
    assertSame(first, manager.getBehaviorTreeLearning(one.toUntyped()));
    assertNotSame(first, second);

    first.begin("sameName", one, "Sequence", "{}");
    second.begin("sameName", two, "Fallback", "{}");
    assertEquals(ActionLearningStatus.NONE, manager.getLearningStatus(one));
    assertEquals(ActionLearningStatus.NONE, manager.getLearningStatus(two));
    first.cancel("sameName");
    assertEquals("{}", second.getBindingsJson("sameName"));
    second.cancel("sameName");
    assertEquals(ActionLearningStatus.NONE, manager.getLearningStatus(one));
    assertEquals(ActionLearningStatus.NONE, manager.getLearningStatus(two));
  }

  @Test
  public void unknownOrMissingActorCannotCreateAnUnownedLearningState() {
    LearningManager manager = new LearningManager();
    try {
      manager.getBehaviorTreeLearning(Factory.createSymbol("unknown"));
      fail("Unknown actor should be rejected");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("Unknown"));
    }
    try {
      manager.getBehaviorTreeLearning(null);
      fail("Missing actor should be rejected");
    } catch (IllegalArgumentException expected) {
      assertTrue(expected.getMessage().contains("actor"));
    }
  }
}
