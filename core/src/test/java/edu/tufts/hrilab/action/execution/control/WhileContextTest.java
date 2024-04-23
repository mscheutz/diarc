/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.Context;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import edu.tufts.hrilab.action.execution.RootContext;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author willie
 */
public class WhileContextTest {
  RootContext root;
  StateMachine sm;

  @Before
  public void initCntext(){
    sm= StateMachine.createTestStateMachine(new HashSet<>());
    root = new RootContext(new ActionConstraints(),sm);
  }

  @After
  public void destroyContext(){
    sm.shutdown();
  }

  @AfterClass
  public static void destroyClass(){
    Database.destroyInstance();
  }

  @Test
  public void testSomeMethod() {
    EventSpec spec1 = new EventSpec(EventSpec.EventType.CONTROL, "while");
    EventSpec spec2 = new EventSpec(EventSpec.EventType.CONTROL, "true");
    EventSpec spec3 = new EventSpec(EventSpec.EventType.CONTROL, "do");
    EventSpec spec5 = new EventSpec(EventSpec.EventType.CONTROL, "endwhile");
    List<EventSpec> specs = Arrays.asList(spec1, spec2, spec3, spec5);
    ScriptParser parser = new ScriptParser(specs);

    // Setup contexts
    Context whileContext = root.setupNextStep(parser);

    // Check while functionality
    whileContext.setStatus(ActionStatus.PROGRESS);
    Context cond = whileContext.getNextStep();
    assertNotNull(cond);              // Check for condition existence
    assertTrue(cond instanceof TrueContext); // check condition is a true context
    cond.setStatus(ActionStatus.PROGRESS);
    cond.setStatus(ActionStatus.SUCCESS);
    assertNull(cond.getNextStep());   // True shouldn't have any next steps
    Context body = whileContext.getNextStep();
    assertNotNull(body);              // Check for body existence
    body.setStatus(ActionStatus.PROGRESS);
    assertNull(body.getNextStep());   // Body shouldn't have any next steps

    // Check that we're back at the beginning of the while (i.e., the condition)
    Context nextIter = whileContext.getNextStep();
    assertNotNull(cond);              // Check for condition existence
    assertTrue(nextIter instanceof TrueContext); // check condition is a true context

    // Note: we should write a more complete test than this.
  }

}
