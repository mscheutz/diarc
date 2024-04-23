/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.RootContext;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class TryCatchTest {
  RootContext root;
  StateMachine sm;

  @BeforeClass
  public static void setupClass() {
    Database.getInstance(); // initialize DB (and DB operators)
  }

  @Before
  public void initContext(){
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
  public void testSuccess() {
    EventSpec spec1 = new EventSpec(EventSpec.EventType.CONTROL, "try");
    EventSpec spec2 = new EventSpec(EventSpec.EventType.CONTROL, "true");
    EventSpec spec3 = new EventSpec(EventSpec.EventType.CONTROL, "catch");
    EventSpec spec4 = new EventSpec(EventSpec.EventType.OPERATOR, "log(\"error\", \"test should not have error here\")");
    EventSpec spec5 = new EventSpec(EventSpec.EventType.CONTROL, "endtry");
    List<EventSpec> specs = Arrays.asList(spec1, spec2, spec3, spec4, spec5);
    ScriptParser parser = new ScriptParser(specs);
    // Setup contexts
    Context tryc = root.setupNextStep(parser);

    // Check try-catch functionality
    Context truec = doStepAndGetNext(tryc);
    assertNotNull(truec);                     // Check for existence of true control inside try block
    assertNull(doStepAndGetNext(truec));   // True shouldn't have any next steps
    assertNull(tryc.getNextStep());    // Try shouldn't have any next steps (we're done)
  }

  @Test
  public void testFailure1() {
    //ActionInterpreter.setDecisionAid(new UtilitarianDecider());
    EventSpec spec1 = new EventSpec(EventSpec.EventType.CONTROL, "try");
    EventSpec spec2 = new EventSpec(EventSpec.EventType.ACTION, "does-not-exist");
    EventSpec spec3 = new EventSpec(EventSpec.EventType.CONTROL, "catch");
    EventSpec spec4 = new EventSpec(EventSpec.EventType.OPERATOR, "log(\"info\", \"Error was expected, everything is OK!\")");
    EventSpec spec5 = new EventSpec(EventSpec.EventType.CONTROL, "endtry");
    List<EventSpec> specs = Arrays.asList(spec1, spec2, spec3, spec4, spec5);
    ScriptParser parser = new ScriptParser(specs);

    // Setup contexts
    Context tryc = root.setupNextStep(parser);

    // Check try-catch functionality
    Context exitc = doStepAndGetNext(tryc);
    assertNotNull(exitc);
    exitc.doStep();
  }

  @Test
  public void testFailure2() {
    EventSpec spec1 = new EventSpec(EventSpec.EventType.CONTROL, "try");
    EventSpec spec2 = new EventSpec(EventSpec.EventType.CONTROL, "exit");
    EventSpec spec3 = new EventSpec(EventSpec.EventType.CONTROL, "catch");
    EventSpec spec4 = new EventSpec(EventSpec.EventType.OPERATOR, "log(\"info\", \"Error was expected, everything is OK!\")");
    EventSpec spec5 = new EventSpec(EventSpec.EventType.CONTROL, "endtry");
    List<EventSpec> specs = Arrays.asList(spec1, spec2, spec3, spec4, spec5);
    ScriptParser parser = new ScriptParser(specs);

    // Setup contexts
    Context tryc = root.setupNextStep(parser);

    // Check try-catch functionality
    tryc.doStep();
    Context exitc = doStepAndGetNext(tryc);
    assertNotNull(exitc);                     // Check for existence of exit context
    exitc.doStep();
  }

  /**
   * Helper method to stand in for StepExecution.
   * @param context
   * @return
   */
  private Context doStepAndGetNext(Context context) {
    context.setStatus(ActionStatus.PROGRESS);
    context.doStep();
    Context next = context.getNextStep();
    return next;
  }

}
