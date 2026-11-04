/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.operators;

import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.db.OperatorDBEntry;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.OperatorContext;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import static org.junit.Assert.*;

public class OperatorsTest {

  private static StateMachine sm;

  public static Object doOperation(String opString) {
    String[] split = opString.split(" ");
    List<String> inputArgs = new ArrayList<>();
    for (int i = 1; i < split.length; ++i) {
      inputArgs.add(split[i]);
    }
    List<String> returnArgs = new ArrayList<>();
    returnArgs.add("!result");

    OperatorDBEntry odb = Database.getOperatorDB().getOperator(split[0]);
    OperatorContext op = new OperatorContext(null, sm, odb, inputArgs, returnArgs);
    op.doStep();
    return op.getArgumentValue("ret");
  }

  @BeforeClass
  public static void setupClass() {
    Database.getInstance();  // initialize DB (and DB operators)
    sm = StateMachine.createTestStateMachine(new HashSet<>());
  }

  @AfterClass
  public static void destroyClass(){
    sm.shutdown();
    Database.destroyInstance();
  }

  @Test
  public void testAdd() {
    assertEquals(1.0+2.0, doOperation("+ 1.0 2.0"));
  }

  @Test
  public void testSubtract() {
    assertEquals(1.0-2.0, doOperation("- 1.0 2.0"));
  }

  @Test
  public void testMultiply() {
    assertEquals(2.0*3.0, doOperation("* 2.0 3.0"));
  }

  @Test
  public void testDivide() {
    assertEquals(2.0/3.0, doOperation("/ 2.0 3.0"));
  }

  @Test
  public void testModulus() {
    assertEquals(10.0%3.0, doOperation("% 10.0 3.0"));
  }

  @Test
  public void testRound() {
    assertEquals(Math.round(3.141592), doOperation("round 3.141592"));
  }

  @Test
  public void testGreater() {
    assertEquals(2>1, doOperation("gt 2 1"));
    assertEquals(1>2, doOperation("gt 1 2"));
    assertEquals(1>1, doOperation("gt 1 1"));
  }

  @Test
  public void testGreaterOrEqual() {
    assertEquals(2>=1, doOperation("ge 2 1"));
    assertEquals(1>=2, doOperation("ge 1 2"));
    assertEquals(1>=1, doOperation("ge 1 1"));
  }

  @Test
  public void testLower() {
    assertEquals(2<1, doOperation("lt 2 1"));
    assertEquals(1<2, doOperation("lt 1 2"));
    assertEquals(1<1, doOperation("lt 1 1"));
  }

  @Test
  public void testLowerOrEqual() {
    assertEquals(2<=1, doOperation("le 2 1"));
    assertEquals(1<=2, doOperation("le 1 2"));
    assertEquals(1<=1, doOperation("le 1 1"));
  }

  @Test
  public void testNot() {
    assertEquals(!true, doOperation("! true"));
    assertEquals(!false, doOperation("! false"));
  }

  @Test
  public void testAnd() {
    assertEquals(true&&false, doOperation("and true false"));
    assertEquals(true&&true, doOperation("and true true"));
  }

  @Test
  public void testOr() {
    assertEquals(true||false, doOperation("|| true false"));
    assertEquals(true||true, doOperation("|| true true"));
  }
}
