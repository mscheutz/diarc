/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.selector;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

import java.util.HashSet;

/**
 * @author willie
 * @author dave
 */
public class UtilitarianActionSelectorTest {
  RootContext root;
 edu.tufts.hrilab.action.state.StateMachine sm;

  @Before
  public void setupClass() {
    sm = StateMachine.createTestStateMachine(new HashSet<>());
    root = new RootContext(new ActionConstraints(),sm);
  }

  @After
  public void destroyClass(){
    sm.shutdown();
  }

  @Test
  public void testLook() {
    edu.tufts.hrilab.action.ActionBuilderRepo.buildLookforAction();
    Predicate predicate = Factory.createPredicate("goal(self,located(ball))");
    ActionConstraints constraints = new ActionConstraints();
    ActionSelector actionSelector = new UtilitarianActionSelector();

    ParameterizedAction action = actionSelector.selectActionForGoal(new edu.tufts.hrilab.action.goal.Goal(predicate), constraints, sm);

    assertNotNull(action);
    assertNotNull(action.getEntry());
    assertNotNull(action.getBindings());
    Database.destroyInstance();
  }

  @Test
  public void testObserve() {
    edu.tufts.hrilab.action.ActionBuilderRepo.buildObsTouching();
    Predicate predicate = Factory.createPredicate("obs(self,touching(X,Y))");
    ActionConstraints constraints = new ActionConstraints();
    ActionSelector actionSelector = new UtilitarianActionSelector();

    ParameterizedAction action = actionSelector.selectActionForGoal(new Goal(predicate), constraints, sm);

    assertNotNull(action);
    assertNotNull(action.getEntry());
    assertNotNull(action.getBindings());
    Database.destroyInstance();
  }

}
