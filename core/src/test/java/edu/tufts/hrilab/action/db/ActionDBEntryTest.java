/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.fol.Factory;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * @author willie
 */
public class ActionDBEntryTest {
  private static final Logger log = LoggerFactory.getLogger(ActionDBEntryTest.class);
  private Integer var = 0;
  private Collection<TRADEServiceInfo> myServices;
  RootContext root;
  StateMachine sm;

  @Before
  public void initDB(){
    Database.getInstance();
    sm= StateMachine.createTestStateMachine(new HashSet<>());
    root = new RootContext(new ActionConstraints(),sm);
  }

  @After
  public void deregister(){
    sm.shutdown();
    Database.destroyInstance();
  }

  public ActionDBEntryTest() {
    try {
      myServices = TRADE.registerAllServices(this,new ArrayList<>());
    } catch (TRADEException e) {
      log.error("[ActionDBEntryTest]",e);
     fail("[ActionDBEntryTest] Exception registering with trade");
    }
  }

  @Test
  public void testCreateActionDBEntry() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("liftup");
    adbeBuilder.addEffect(new Effect(Factory.createPredicate("lifted(?arm)"), EffectType.ALWAYS));
    adbeBuilder.addRole(new ActionBinding.Builder("?lifter", Object.class).setDefaultValue("self").build());
    adbeBuilder.addRole(new ActionBinding.Builder("?arm", Object.class).build());
    ActionDBEntry adb = adbeBuilder.build(true);

    // 3 roles bc ?actor is always an explicit role, even if it's not explicitly added
    assertEquals(3, adb.getNumRoles(), 0.0);
    assertNotNull(Database.getActionDB().getAction("liftup"));
  }

  @Test
  public void testInvokeAction() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("testMethod");
    List<TRADEServiceInfo> testMethodTSIs = myServices.stream().filter(t -> t.serviceName.equals("testMethod")).collect(Collectors.toList());
    assertEquals(testMethodTSIs.size(),1);
    adbeBuilder.setActionImplementer(testMethodTSIs.get(0));
    ActionDBEntry adb = adbeBuilder.build(false);
    adb.executeAction(new ArrayList<>());
    assertEquals(1, this.var, 0);
    //     adb.executeAction(mock, null, true);
    //     assertEquals(1, this.var, 0);
  }

  @Test
  public void testArguments() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("testMethod2");
    List<TRADEServiceInfo> testMethodTSIs = myServices.stream().filter(t -> t.serviceName.equals("testMethod2")).collect(Collectors.toList());
    assertEquals(testMethodTSIs.size(),1);
    adbeBuilder.setActionImplementer(testMethodTSIs.get(0));
    ActionDBEntry adb = adbeBuilder.build(false);

    Collection<ActionBinding> args = new ArrayList<>();
    ActionBinding binding = new ActionBinding.Builder("firstTest", Integer.class).setDefaultValue("5").build();
    binding.bind(5);
    args.add(binding);
    adb.executeAction(args);
  }


  /*
  @Test
  public void bindString() {
      System.out.println("bindString");
      ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("testMethod", "primitive");
      TestClass obj = new TestClass();
      MockActionImplementer mock = new MockActionImplementer();
      mock.obj = obj;
      try {
        mock.delegate = obj.getClass().getMethod("testMethod");
      } catch (NoSuchMethodException | SecurityException ex) {
          fail(ex.getMessage());
      }
      HashSet<String> groups = new HashSet<>();
      groups.add("group1");
      adbeBuilder.setActionImplementer("componentFoo", null, groups, "testMethod");
      ActionDBEntry adb = adbeBuilder.build();

      Collection<ActionBinding> args = new ArrayList<>();
      ActionBinding binding = new ActionBinding("firstTest", "java.lang.String", "def", true);
      assert(!binding.isBound());
      args.add(binding);
      adb.executeAction(mock, args);
//        assertEquals(1, obj.var, 0);
      assertTrue(binding.isBound());
      assertEquals("firstTest", binding.getBindingNameDeep());
      assertEquals("java.lang.String", binding.getBinding().getClass().getName());
      assertEquals("1", binding.getBinding());
      System.out.println(binding.getBinding());
  }
  @Test
  public void bindBoolean() {
      System.out.println("bindBoolean");
      ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("testMethod", "primitive");
      TestClass obj = new TestClass();
      MockActionImplementer mock = new MockActionImplementer();
      mock.obj = obj;
      try {
        mock.delegate = obj.getClass().getMethod("testMethod");
      } catch (NoSuchMethodException | SecurityException ex) {
          fail(ex.getMessage());
      }
      HashSet<String> groups = new HashSet<>();
      groups.add("group1");
      adbeBuilder.setActionImplementer("componentFoo", null, groups, "testMethod");
      ActionDBEntry adb = adbeBuilder.build();

      Collection<ActionBinding> args = new ArrayList<>();
      ActionBinding binding2 = new ActionBinding("secondTest", "java.lang.Boolean", "false", true);
      assert(!binding2.isBound());
      args.add(binding2);
      adb.executeAction(mock, args, true);
      assertTrue(binding2.isBound());
      assertEquals("secondTest", binding2.getBindingNameDeep());
      assertEquals("java.lang.Boolean", binding2.getBinding().getClass().getName());
//        assertTrue(binding2.getBinding());
      System.out.println(binding2.getBinding());
}

  @Test
  public void bindDouble() {
      System.out.println("bindDouble");
      ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("testMethod", "primitive");
      TestClass obj = new TestClass();
      MockActionImplementer mock = new MockActionImplementer();
      mock.obj = obj;
      try {
        mock.delegate = obj.getClass().getMethod("testMethod");
      } catch (NoSuchMethodException | SecurityException ex) {
          fail(ex.getMessage());
      }
      HashSet<String> groups = new HashSet<>();
      groups.add("group1");
      adbeBuilder.setActionImplementer("componentFoo", null, groups, "testMethod");
      ActionDBEntry adb = adbeBuilder.build();

      Collection<ActionBinding> args = new ArrayList<>();
      ActionBinding binding2 = new ActionBinding("thirdTest", "java.lang.Double", "false", true);
      assert(!binding2.isBound());
      args.add(binding2);
      adb.executeAction(mock, args, true);
      assertTrue(binding2.isBound());
      assertEquals("thirdTest", binding2.getBindingNameDeep());
      assertEquals("java.lang.Double", binding2.getBinding().getClass().getName());
//        assert(binding2.getBinding());

  }
*/
  @Test
  public void testEventSpecs() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("liftup");
    adbeBuilder.addEventSpec(new EventSpec(EventSpec.EventType.ACTION, "go nowhere"));
    adbeBuilder.addEventSpec(new EventSpec(EventSpec.EventType.ACTION, "stop \"everything now\""));
    ActionDBEntry adb = adbeBuilder.build(false);

    assertEquals(2, adb.getEventSpecs().size(), 0);
    assertEquals("go", adb.getEventSpec(0).getCommand());
    assertEquals("stop", adb.getEventSpec(1).getCommand());
    assertEquals(1, adb.getEventSpec(1).getInputArgs().size());
    assertEquals("stop", adb.getEventSpec("stop nowhere", 0).getCommand());
  }

  @Test
  public void testPreconditions() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("liftup");
    adbeBuilder.addCondition(new Condition(Factory.createPredicate("hasArm(left)"), ConditionType.PRE));
    ActionDBEntry adb = adbeBuilder.build(false);
    assertEquals(1, adb.getPreConditions().size(), 0);
  }

  @Test
  public void testEffects() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("liftup");

    Effect raised = new Effect(Factory.createPredicate("raised(arm)"), EffectType.ALWAYS);
    Effect lifted = new Effect(Factory.createPredicate("lifted(arm)"), EffectType.SUCCESS);
    Effect notLifted = new Effect(Factory.createPredicate("not(lifted(arm))"), EffectType.FAILURE);
    Effect unknown = new Effect(Factory.createPredicate("unknown(raised(arm))"), EffectType.NONPERF);
    adbeBuilder.addEffect(raised);
    adbeBuilder.addEffect(lifted);
    adbeBuilder.addEffect(notLifted);
    adbeBuilder.addEffect(unknown);
    ActionDBEntry adb = adbeBuilder.build(false);

    assertEquals(3, adb.getPostConditions().size()); // only ALWAYS and SUCCESS count as post-conditions

    // Effects in case of SUCCESS should also contain ALWAYS effects.
    assertTrue(adb.getEffects(ActionStatus.SUCCESS).contains(raised));
    assertTrue(adb.getEffects(ActionStatus.SUCCESS).contains(lifted));
    assertTrue(adb.getEffects(ActionStatus.FAIL).contains(notLifted));
    assertTrue(adb.getEffects(ActionStatus.CANCEL).contains(unknown));
  }

  /**
   * Test of addRoles method and other methods related to roles.
   */
  @Test
  public void testRoles() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("liftup");
    adbeBuilder.addRole(new ActionBinding.Builder("?a", String.class).setDefaultValue("default").setIsReturn(true).build());
    adbeBuilder.addRole(new ActionBinding.Builder("?b", String.class).setIsReturn(true).build());
    adbeBuilder.addRole(new ActionBinding.Builder("?c", String.class).setDefaultValue("default").setIsReturn(true).build());
    adbeBuilder.addRole(new ActionBinding.Builder("?d", String.class).setDefaultValue("default").build());
    adbeBuilder.addRole(new ActionBinding.Builder("?e", int.class).build());
    adbeBuilder.addRole(new ActionBinding.Builder("?f", double.class).build());
    ActionDBEntry adb = adbeBuilder.build(false);

    assertEquals("?actor", adb.getRoleName(0));
    assertEquals("?a", adb.getRoleName(1));
    assertEquals("?b", adb.getRoleName(2));
    assertEquals("?c", adb.getRoleName(3));
    assertEquals("?d", adb.getRoleName(4));
    assertEquals("?e", adb.getRoleName(5));
    assertEquals("?f", adb.getRoleName(6));
    assertEquals(Symbol.class, adb.getRoleJavaType(0));
    assertEquals(String.class, adb.getRoleJavaType(1));
    assertEquals(7, adb.getNumRoles(), 0);
  }

  /**
   * Test of getType method, of class ActionDBEntry.
   */
  @Test
  public void testGetType() {
    ActionDBEntry.Builder adbeBuilder = new ActionDBEntry.Builder("liftup");
    ActionDBEntry adb = adbeBuilder.build(false);
    assertEquals("liftup", adb.getType());
  }

  @TRADEService
  public void testMethod() {
    var = 1;
  }

  @TRADEService
  public void testMethod2(Integer toSet) {
    var = toSet;
  }
}
