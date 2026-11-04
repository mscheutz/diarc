/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.state;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import java.util.HashSet;
import java.util.Set;

import edu.tufts.hrilab.action.execution.RootContext;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author willie
 */
public class StateTest {
  edu.tufts.hrilab.action.state.StateMachine sm;
  RootContext rootContext;

  @BeforeClass
  public static void initDB(){
    edu.tufts.hrilab.action.ActionBuilderRepo.buildLookforAction();
  }

  @AfterClass
  public static void deregister(){
    Database.destroyInstance();
  }

  @Before
  public void initSM(){
    sm= StateMachine.createTestStateMachine(new HashSet<>());
    rootContext = new RootContext(new ActionConstraints(),sm);
  }

  @After
  public void destroySM(){
    sm.shutdown();
  }

  /**
   * Test of createRoot method, of class State.
   */
  @Test
  public void testCreateRoot() {
    edu.tufts.hrilab.action.state.State root = edu.tufts.hrilab.action.state.State.createRoot(new HashSet<>());
    assertNotNull(root);
  }

  @Test
  public void testCreateRoot_null() {
    edu.tufts.hrilab.action.state.State root = edu.tufts.hrilab.action.state.State.createRoot();
    assertNotNull(root);
  }

  @Test
  public void testCreateRoot_init1() {
    Set s = new HashSet<Predicate>();
    s.add(new Predicate("foo", "bar", "baz"));
    edu.tufts.hrilab.action.state.State root = edu.tufts.hrilab.action.state.State.createRoot(s);
    assertNotNull(root);
  }

  /**
   * Test that the update creates a new state and that the new state is not null
   * and is not the same as the previous state
   */
  @Test
  public void testUpdate() {
    edu.tufts.hrilab.action.state.State root = edu.tufts.hrilab.action.state.State.createRoot(new HashSet<>());
    assertNotNull(root);
    Context context = rootContext.addEvent(new EventSpec(EventSpec.EventType.ACTION, "self.lookfor(loc1, ball)"));
    context.setStatus(ActionStatus.SUCCESS);
    edu.tufts.hrilab.action.state.State next = root.update(context);
    assertNotNull(next);
    assertNotSame(root, next);
  }
  
  /**
   * Test that the update creates a new state and that the new state includes
   * the desired predicate
   */
  @Test
  public void testHasFact() {
    edu.tufts.hrilab.action.state.State root = edu.tufts.hrilab.action.state.State.createRoot(new HashSet<>());
    assertNotNull(root);
    Context context = rootContext.addEvent(new EventSpec(EventSpec.EventType.ACTION, "self.lookfor(loc1, ball)"));
    context.setStatus(ActionStatus.SUCCESS);
    edu.tufts.hrilab.action.state.State next = root.update(context);
    assertNotNull(next);
    assertTrue(next.hasFact(Factory.createPredicate("located(ball)")));
  }

    /**
   * Test that the update creates a new state and that the new state does not
   * include a predicate because the action failed.
   */
  @Test
  public void testHasFact_ActionFailed() {
    edu.tufts.hrilab.action.state.State root = edu.tufts.hrilab.action.state.State.createRoot(new HashSet<>());
    assertNotNull(root);
    Context context = rootContext.addEvent(new EventSpec(EventSpec.EventType.ACTION, "self.lookfor(loc1, ball)"));
    context.setStatus(ActionStatus.FAIL);
    State next = root.update(context);
    assertNotNull(next);
    assertFalse(next.hasFact(Factory.createPredicate("located(ball)")));
  }

}
