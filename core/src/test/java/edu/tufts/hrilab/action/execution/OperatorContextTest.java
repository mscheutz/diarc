/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.db.OperatorDBEntry;
import edu.tufts.hrilab.action.state.StateMachine;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import static org.junit.Assert.*;

public class OperatorContextTest {

  private static OperatorContext op;
  private static StateMachine sm;

  public static boolean dummyOperator(int a, double b) {
    return a>b;
  }

  @BeforeClass
  public static void setupClass() {
    sm = StateMachine.createTestStateMachine(new HashSet<>());

    try {
      Method m = OperatorContextTest.class.getMethod("dummyOperator", int.class, double.class);
      OperatorDBEntry odb = new OperatorDBEntry(m.getName(), m);
      List<String> inputArgs = new ArrayList<>();
      inputArgs.add("2");
      inputArgs.add("1.0");
      List<String> returnArgs = new ArrayList<>();
      returnArgs.add("!ret");

      op = new OperatorContext(null, sm,odb, inputArgs, returnArgs);
    }
    catch (NoSuchMethodException e) {
      System.out.println("Failed to get dummyOperator. " + e);
    }
  }

  @AfterClass
  public static void destroy(){
    Database.destroyInstance();
  }

  @Test
  public void testSetupArguments() {
    assertEquals(int.class, op.getArgumentType("a"));
    assertEquals(double.class, op.getArgumentType("b"));
    assertEquals(boolean.class, op.getArgumentType("ret"));
  }

  @Test
  public void testDoStep() {
    op.doStep();
    assertEquals(true, op.getArgumentValue("ret"));
  }
}
