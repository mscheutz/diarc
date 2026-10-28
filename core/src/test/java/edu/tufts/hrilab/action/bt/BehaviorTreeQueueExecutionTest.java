/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.bt;

import org.junit.BeforeClass;
import org.junit.Test;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.fol.Factory;

import static org.junit.Assert.*;

/** Run the same semantics/lifecycle cases while the root holds queued resources. */
public class BehaviorTreeQueueExecutionTest extends BehaviorTreeExecutionTest {
  @BeforeClass
  public static void initialize() {
    initializeManager("edu.tufts.hrilab.action.manager.QueueExecutionManager");
  }

  @Test(timeout = 15000)
  public void higherPriorityGoalBlocksAndThenResumesTree() throws Exception {
    long tree = submit("<Delay delay_msec=\"200\"><Record label=\"tree\"/></Delay>");
    prepare("priority");
    long priority = track(manager.submitGoal(Factory.createPredicate("btWork(self:agent,priority)"),
        ExecutionType.ACT, PriorityTier.URGENT, 9999));
    waitStarted("priority");
    Thread.sleep(250);
    assertFalse(trace.contains("record:tree"));
    release("priority");
    succeeds(priority);
    succeeds(tree);
    assertTrue(trace.indexOf("end:priority") < trace.indexOf("record:tree"));
  }
}
