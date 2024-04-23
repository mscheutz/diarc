/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.lock.ActionResourceLock;
import edu.tufts.hrilab.action.lock.ActionResourceLockLinear;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import java.util.HashSet;
import java.util.Set;
import java.util.Stack;

import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Test;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;

/**
 *
 * @author willie
 */
public class ActionContextTest {

  private RootContext root;
  private StateMachine sm;
  private Symbol actor = Factory.createSymbol("self:agent");

  public ActionContextTest() {
  }

  @Before
  public void reset() {
    Set<Predicate> s = new HashSet<>();
    s.add(Factory.createPredicate("at(default)"));
    sm = StateMachine.createTestStateMachine(s);
    root = new RootContext(new ActionConstraints(),sm);
  }

  @After
  public void deregisterSM(){
    sm.shutdown();
  }

  @BeforeClass
  public static void setupClass() {
    Database.getInstance();
    ActionResourceLock.motionLock = new ActionResourceLockLinear("motionLock");

    ActionDBEntry.Builder lookforBuilder = new ActionDBEntry.Builder("lookfor");
    lookforBuilder.addRole(new ActionBinding.Builder("?loc",edu.tufts.hrilab.fol.Symbol.class).build());
    lookforBuilder.addRole(new ActionBinding.Builder("?obj",edu.tufts.hrilab.fol.Symbol.class).build());
    lookforBuilder.addResourceLock("motionLock");
    lookforBuilder.setCost("0.23");
    lookforBuilder.addEffect(new Effect(Factory.createPredicate("located(?loc,?obj)"), EffectType.SUCCESS));
    lookforBuilder.addEffect(new Effect(Factory.createPredicate("located(?obj)"), EffectType.SUCCESS));
    lookforBuilder.build(true);

    ActionDBEntry.Builder movetoBuilder = new ActionDBEntry.Builder("moveto");
    movetoBuilder.addRole(new ActionBinding.Builder("?location", edu.tufts.hrilab.fol.Symbol.class).setDefaultValue("default").build());
    movetoBuilder.addRole(new ActionBinding.Builder("?start", edu.tufts.hrilab.fol.Symbol.class).setDefaultValue("default").build());
    movetoBuilder.addEffect(new Effect(Factory.createPredicate("at(?location)"), EffectType.SUCCESS));
    Condition pc3 = new Condition(Factory.createPredicate("at(?start)"), ConditionType.PRE);
    movetoBuilder.addCondition(pc3);
    movetoBuilder.build(true);

    ActionDBEntry.Builder findBuilder = new ActionDBEntry.Builder("findinvision");
    findBuilder.addRole(new ActionBinding.Builder("?object",edu.tufts.hrilab.fol.Symbol.class).build());
    findBuilder.build(true);
  }

  @AfterClass
  public static void deregister(){
    Database.destroyInstance();
  }

  /**
   * Add a new "searchfor" action to the DB. This is nearly identical to the
   * "lookfor" action created in the setupClass method except it has a sequence
   * of eventSpecs added and also has a different post-condition to distinguish
   * it from the "lookfor" post-condition. This is needed to test the event spec
   * functionality in testAddEventFromScript.
   */
  private void addSearchForActionWithEventSpecs() {
    ActionDBEntry.Builder searchforBuilder = new ActionDBEntry.Builder("searchfor");
    searchforBuilder.addRole(new ActionBinding.Builder("?loc",edu.tufts.hrilab.fol.Symbol.class).build());
    searchforBuilder.addRole(new ActionBinding.Builder("?obj",edu.tufts.hrilab.fol.Symbol.class).build());
    searchforBuilder.addResourceLock("motionLock");
    searchforBuilder.setCost("0.23");

    searchforBuilder.addEventSpec(new EventSpec(EventSpec.EventType.CONTROL, "if"));
    searchforBuilder.addEventSpec(new EventSpec(EventSpec.EventType.ACTION, "?actor.moveto ?loc origin"));
    searchforBuilder.addEventSpec(new EventSpec(EventSpec.EventType.CONTROL, "then"));
    searchforBuilder.addEventSpec(new EventSpec(EventSpec.EventType.ACTION, "?actor.findinvision ?obj"));
    searchforBuilder.addEventSpec(new EventSpec(EventSpec.EventType.CONTROL, "endif"));

    searchforBuilder.addEffect(new Effect(Factory.createPredicate("found(?obj)"), EffectType.ALWAYS));
    searchforBuilder.build(true);
  }

  private Context setupActionContext(Predicate goalPredicate) {
    Goal g = new Goal(actor, goalPredicate);
    Context goalContext = root.addEvent(g, sm);
    goalContext.setupNextStep(); // perform action selection for goal
    Context actionContext = goalContext.getChildContexts().get(0);
    assertNotNull(actionContext);
    return actionContext;
  }

  private Context setupUnallowedAction() {
    ActionDBEntry.Builder unallowedBuilder = new ActionDBEntry.Builder("hit");
    unallowedBuilder.addRole(new ActionBinding.Builder("?target",edu.tufts.hrilab.fol.Symbol.class).build());
    unallowedBuilder.addResourceLock("motionLock");
    unallowedBuilder.setCost("0.23");

    unallowedBuilder.addEffect(new Effect(Factory.createPredicate("moved(?target)"), EffectType.ALWAYS));
    unallowedBuilder.addEffect(new Effect(Factory.createPredicate("harmed(?target)"), EffectType.ALWAYS));
    unallowedBuilder.build(true);

    Context hitContext = setupActionContext(Factory.createPredicate("moved(person)"));
    return hitContext;
  }

  @Test
  public void testAddEventFromSpec() {
    Context lookforContext = setupActionContext(Factory.createPredicate("located(home,ball)"));

    assertEquals(Factory.createSymbol("self:agent"), lookforContext.getArgumentValue("?actor"));
    assertEquals(Factory.createSymbol("home"), lookforContext.getArgumentValue("?loc"));
    assertEquals(Factory.createSymbol("ball"), lookforContext.getArgumentValue("?obj"));

    EventSpec event = new EventSpec(EventSpec.EventType.ACTION, "moveto(?loc, ?obj)");
    ActionContext moveContext = (ActionContext)lookforContext.addEvent(event);

    // note that these are using the argument names as defined by their roles
    assertEquals(Factory.createSymbol("self:agent"), moveContext.getArgumentValue("?actor"));
    assertEquals(Factory.createSymbol("home"), moveContext.getArgumentValue("?location"));
    assertEquals(Factory.createSymbol("ball"), moveContext.getArgumentValue("?start"));

    //TODO: the fact thay you can't specify type here is maybe why we don't want to use this syntax
    EventSpec event2 = new EventSpec(EventSpec.EventType.ACTION, "self.moveto(?obj, ?loc)");
    ActionContext moveContext2 = (ActionContext)lookforContext.addEvent(event2);

    // note that these are using the argument names as defined by their roles
    assertEquals(Factory.createSymbol("self"), moveContext2.getArgumentValue("?actor"));
    assertEquals(Factory.createSymbol("ball"), moveContext2.getArgumentValue("?location"));
  }

  @Test
  public void testAddEventFromScript() {
    addSearchForActionWithEventSpecs();
    Context searchforContext = setupActionContext(Factory.createPredicate("found(ball)"));

    searchforContext.setStatus(ActionStatus.PROGRESS); // in lieu of using StepExecution
    Context ifStep = searchforContext.getNextStep();
    assertEquals(1, searchforContext.childContexts.size());
    assertNotNull(ifStep);
    ifStep.setStatus(ActionStatus.PROGRESS); // in lieu of using StepExecution
    Context condStep = ifStep.getNextStep();
    assertNotNull(condStep);
    condStep.setStatus(ActionStatus.PROGRESS); // in lieu of using StepExecution
    // but I'm not sure how to check for that right now
    Context none = condStep.getNextStep();
    assertNull(none);
    Context thenblock = ifStep.getNextStep();
    // thenblock is null because the condition failed (because method for action was not setup)
    //thenblock.doStep(aci);
    //Context thenstep1 = thenblock.getNextStep();
    //assertNotNull(thenstep1);
    //assertTrue(thenstep1 instanceof ActionContext);
    assertNull(thenblock);
    assertNull(ifStep.getNextStep());
  }

  @Test
  public void testAddEventFromGoal() {
    Context lookforContext = setupActionContext(Factory.createPredicate("located(home,ball)"));

    assertEquals(Factory.createSymbol("self:agent"), lookforContext.getArgumentValue("?actor"));
    assertEquals(Factory.createSymbol("home"), lookforContext.getArgumentValue("?loc"));
    assertEquals(Factory.createSymbol("ball"), lookforContext.getArgumentValue("?obj"));
  }
  
  @Test
  public void testAddEventFromGoalNoAllowedAction() {
    root.getConstraints().addForbiddenAction("hit");
    Context badContext = setupUnallowedAction();
    assertTrue(badContext instanceof FailureContext);
  }

  @Ignore
  @Test
  public void testResourceLocks() {
    Context lookforContext = setupActionContext(Factory.createPredicate("located(ball)"));

    Goal g = new Goal(actor, Factory.createPredicate("located(ball)"));
    ActionInterpreter ai = ActionInterpreter.createInterpreterFromGoal(g, root, sm, ExecutionType.ACT);
    assertNotNull(ai);
    assertTrue(lookforContext.acquireLocks(ai, true));

    ActionResourceLock lock = ActionResourceLock.getLock("motionLock");
    assertEquals(lock.owner.peek(), ai);
  }

  @Test
  public void testGetArgument() {
    Context lookforContext = setupActionContext(Factory.createPredicate("located(home,ball)"));

    lookforContext.doStep();

    assertEquals(Factory.createSymbol("self:agent"), lookforContext.getArgumentValue("?actor"));
    assertEquals(Factory.createSymbol("home"), lookforContext.getArgumentValue("?loc"));
    assertEquals(Factory.createSymbol("ball"), lookforContext.getArgumentValue("?obj"));

  }

  @Test
  public void testPriorityParameters() {
    Context lookforContext = setupActionContext(Factory.createPredicate("located(ball)"));
    assertTrue(lookforContext instanceof ActionContext);
    assertEquals(0.23, ((ActionContext)lookforContext).getCost(), 0.0);
  }

  @Test
  public void testIsNotApproved() {
    Context moveContext = setupActionContext(Factory.createPredicate("at(home)"));
    assertNotNull(moveContext);

    sm.retractBelief(Factory.createPredicate("at(default)"));
    sm.assertBelief(Factory.createPredicate("at(foo)"));
    assertFalse(moveContext.isApproved().getValue());
  }

  @Test
  public void testIsApproved() {
    Context moveContext = setupActionContext(Factory.createPredicate("at(home)"));
    assertNotNull(moveContext);

    sm.retractBelief(Factory.createPredicate("at(default)"));
    sm.assertBelief(Factory.createPredicate("at(foo)"));
    assertFalse(moveContext.isApproved().getValue());

    sm.retractBelief(Factory.createPredicate("at(foo)"));
    sm.assertBelief(Factory.createPredicate("at(default)"));
    assertTrue(moveContext.isApproved().getValue());
  }

  @Ignore
  @Test
  public void testUpdateArguments() {
    Context lookforContext = setupActionContext(Factory.createPredicate("located(ball)"));

    assertNull(lookforContext.getArgumentValue("?actor"));
    assertNull(lookforContext.getArgumentValue("?loc"));
    assertEquals("ball", lookforContext.getArgumentValue("?obj"));

    lookforContext.doStep();

    assertEquals("self", lookforContext.getArgumentValue("?actor"));
    assertEquals("home", lookforContext.getArgumentValue("?loc"));
    assertEquals("ball", lookforContext.getArgumentValue("?obj"));

  }

  @Test
  public void testPersistent() {
    ActionDBEntry.Builder dummyAction = new ActionDBEntry.Builder("somescript");
    Predicate pc = Factory.createPredicate("someeffect");
    dummyAction.addEffect(new Effect(pc, EffectType.ALWAYS));
    dummyAction.addEventSpec(new EventSpec(EventSpec.EventType.CONTROL, "true"));
    dummyAction.build(true);

    Context context = setupActionContext(Factory.createPredicate("persistent", pc));

    Stack<Context> stack = new Stack<>();
    for(int i=0; i<5; i++) {
      stack.push(context);
      context.doStep();
      context = context.getNextStepForType();
      if(context == null && !stack.isEmpty()) {
        context = stack.pop();
      }
      assertNotNull(context);
    }

    ActionDBEntry.Builder dummyAction2 = new ActionDBEntry.Builder("someprimitive");
    Predicate pc2 = Factory.createPredicate("someothereffect");
    dummyAction2.addEffect(new Effect(pc2, EffectType.ALWAYS));
    dummyAction2.build(true);

    Context context2 = setupActionContext(Factory.createPredicate("persistent", pc2));

    Stack<Context> stack2 = new Stack<>();
    for(int i=0; i<5; i++) {
      stack2.push(context2);
      context2.doStep();
      context2 = context2.getNextStepForType();
      if(context2 == null && !stack2.isEmpty()) {
        context2 = stack2.pop();
      }
      assertNotNull(context2);
    }
  }
}
