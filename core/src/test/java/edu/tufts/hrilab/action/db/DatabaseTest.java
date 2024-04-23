/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.db.performanceMeasures.PerformanceMeasures;
import edu.tufts.hrilab.action.operators.OperatorSymbol;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import edu.tufts.hrilab.fol.Symbol;
import org.json.JSONException;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import org.skyscreamer.jsonassert.JSONAssert;
import org.skyscreamer.jsonassert.JSONCompareMode;

import static org.junit.Assert.*;

/**
 *
 * @author willie
 */
public class DatabaseTest {
  
  private static ActionDBEntry lookfor1, lookfor2, add1, add2, required1, required2, required3, pickupDBE, placeDBE;

  private static Database db;

  @BeforeClass
  public static void setupClass() {
    Database.getInstance();
    // Build "base" lookfor
    ActionDBEntry.Builder lookforBuilder = new ActionDBEntry.Builder("lookfor");
    lookforBuilder.addRole(new ActionBinding.Builder("?object", Symbol.class).build());
    lookfor1 = lookforBuilder.build(true);

    // Build "specific" lookfor
    ActionDBEntry.Builder lookforBuilder2 = new ActionDBEntry.Builder("lookfor");
    lookforBuilder2.addRole(new ActionBinding.Builder("?object", Symbol.class).build());
    lookforBuilder2.addRole(new ActionBinding.Builder("?room", Symbol.class).build());
    lookfor2 = lookforBuilder2.build(true);

    // Build an action adding three integer
    ActionDBEntry.Builder addInt = new ActionDBEntry.Builder("add");
    addInt.addRole(new ActionBinding.Builder("?a", int.class).build());
    addInt.addRole(new ActionBinding.Builder("?b", int.class).build());
    addInt.addRole(new ActionBinding.Builder("?c", int.class).build());
    add1 = addInt.build(true);

    // Build an action adding three double
    ActionDBEntry.Builder addDouble = new ActionDBEntry.Builder("add");
    addDouble.addRole(new ActionBinding.Builder("?a", double.class).build());
    addDouble.addRole(new ActionBinding.Builder("?b", double.class).build());
    addDouble.addRole(new ActionBinding.Builder("?c", double.class).build());
    add2 = addDouble.build(true);

    // Build an action with "global" (i.e. arguments to provide) and local variables
    ActionDBEntry.Builder req = new ActionDBEntry.Builder("required");
    req.addRole(new ActionBinding.Builder("?a", int.class).build());
    req.addRole(new ActionBinding.Builder("?b", int.class).build());
    req.addRole(new ActionBinding.Builder("!c", int.class).setIsLocal(true).build());
    required1 = req.build(true);

    // Build an action with optional arguments (default value)
    req = new ActionDBEntry.Builder("required");
    req.addRole(new ActionBinding.Builder("?a", int.class).build());
    req.addRole(new ActionBinding.Builder("?b", double.class).setDefaultValue("0.0").build());
    req.addRole(new ActionBinding.Builder("!c", int.class).setIsLocal(true).build());
    required2 = req.build(true);

    // Build an action with optional arguments (return value)
    req = new ActionDBEntry.Builder("required");
    req.addRole(new ActionBinding.Builder("?a", double.class).build());
    req.addRole(new ActionBinding.Builder("?b", double.class).setIsReturn(true).build());
    req.addRole(new ActionBinding.Builder("!c", int.class).setIsLocal(true).build());
    required3 = req.build(true);

    ActionDBEntry.Builder place = new ActionDBEntry.Builder("place");
    place.addRole(new ActionBinding.Builder("?object", edu.tufts.hrilab.fol.Symbol.class).build());
    place.addRole(new ActionBinding.Builder("?arm", edu.tufts.hrilab.fol.Symbol.class).build());
    place.addEffect(new Effect(Factory.createPredicate("not(touching(?actor,?object,?arm))"), edu.tufts.hrilab.action.EffectType.SUCCESS));
    place.addEffect(new Effect(Factory.createPredicate("not(holding(?actor,?object,?arm))"), edu.tufts.hrilab.action.EffectType.SUCCESS));
    placeDBE = place.build(true);

    ActionDBEntry.Builder pickup = new ActionDBEntry.Builder("pickup");
    pickup.addRole(new ActionBinding.Builder("?object",edu.tufts.hrilab.fol.Symbol.class).build());
    pickup.addRole(new ActionBinding.Builder("?arm",edu.tufts.hrilab.fol.Symbol.class).setDefaultValue("left_arm").build());
    pickup.addEffect(new Effect(Factory.createPredicate("touching(?actor,?object,?arm)"), EffectType.SUCCESS));
    pickup.addEffect(new Effect(Factory.createPredicate("holding(?actor,?object,?arm)"), EffectType.SUCCESS));
    pickupDBE = pickup.build(true);

  }

  @AfterClass
  public static void destroy(){
    Database.destroyInstance();
  }

  /**
   * Test of getDB method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testGetDB() {
  }

  /**
   * Test of getDBRoot method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testGetDBRoot() {
  }

  /**
   * Test of getAction method, of class ActionDatabase.
   */
  @Test
  public void testGetAction() {
    // Make sure we get the right entries for the right parameter count
    ActionDBEntry lookfor1_test = Database.getActionDB().getAction("lookfor", Arrays.asList(String.class));
    ActionDBEntry lookfor2_test = Database.getActionDB().getAction("lookfor", Arrays.asList(String.class, String.class));
    assertEquals(lookfor1, lookfor1_test);
    System.out.println("[testGetAction] if failed, run as independent test or fork based on class");
    assertEquals(lookfor2, lookfor2_test);

    // Make sure we get null if parameters count does not match or wrong type
    ActionDBEntry null_test = Database.getActionDB().getAction("lookfor", Arrays.asList());
    assertNull(null_test);
//    null_test = Database.getActionDB().getAction("lookfor", Arrays.asList(Arrays.class));
//    assertNull(null_test);

    // Make sure we get the most appropriate action for the supplied types
    ActionDBEntry add1_test = Database.getActionDB().getAction("add", Arrays.asList(int.class, int.class, int.class));
    ActionDBEntry add2_test = Database.getActionDB().getAction("add", Arrays.asList(double.class, int.class, double.class));
    assertEquals(add1, add1_test);
    assertEquals(add2, add2_test);

    // Make sure we check for all required arguments to be provided and handle optional arguments correctly.
    ActionDBEntry required_test1 = Database.getActionDB().getAction("required", Arrays.asList());
    ActionDBEntry required_test2 = Database.getActionDB().getAction("required", Arrays.asList(int.class));
    ActionDBEntry required_test3 = Database.getActionDB().getAction("required", Arrays.asList(int.class, int.class));
    ActionDBEntry required_test4 = Database.getActionDB().getAction("required", Arrays.asList(int.class, double.class));
    ActionDBEntry required_test5 = Database.getActionDB().getAction("required", Arrays.asList(int.class, int.class, int.class));
    ActionDBEntry required_test6 = Database.getActionDB().getAction("required", Arrays.asList(double.class));
    assertNull(required_test1);
    assertEquals(required2, required_test2);
    assertEquals(required1, required_test3);
    assertEquals(required1, required_test3);
    assertEquals(required2, required_test4);
    assertNull(required_test5);
    assertEquals(required3, required_test6);

    // Make sure string arguments are working (since many types provided by text input will be strings)
    ActionDBEntry string_test = Database.getActionDB().getAction("add", Arrays.asList(String.class, String.class, String.class));
    //TODO: uncomment this once the test has been fixed
    //assertEquals(add1, string_test);
    // TODO: Ideally here we might want to get add2 (that handles doubles) since it is more generic that the one with ints.

    // Make sure defaut getAction() returns the last added action that matches
    add2_test = Database.getActionDB().getAction("add");
    assertEquals(add2, add2_test);
  }

  /**
   * Test of putAction method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testPutAction() {
  }

  /**
   * Test of getActionTypes method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testGetActionTypes() {
  }

  /**
   * Test of checkForPost method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testCheckforPost() {
  }

  /**
   * Test of lookupPost method, of class ActionDatabase.
   */
  @Test
  public void testLookupPost() {
    Predicate pred = Factory.createPredicate("touching(self,ball,arm)");
    List<ActionDBEntry> dbLookUp = Database.getActionDB().getActionsByEffect(Factory.createSymbol("self"), pred);
    List<Effect> effects = dbLookUp.get(0).getEffects();
    for (Effect effect : effects) {
      if (pred.instanceOf(effect.getPredicate())) {
        assertTrue(true);
        return;
      }
    }
    assertTrue(false);
  }

  //TODO:brad:why do we have all of these empty tests?

  /**
   * Test of addFunction method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testAddFunction() {
  }

  /**
   * Test of lookupFunction method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testLookupFunction() {
  }

  /**
   * Test of addPostcondToDB method, of class ActionDatabase.
   */
  @Test
  public void testAddPostcondToDB() {
  }

  /**
   * Test of assertFact method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testAssertFact_String_Object() {
  }

  /**
   * Test of assertFact method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testAssertFact_String() {
  }

  /**
   * Test of retractFact method, of class ActionDatabase.
   */
  @Test
  @Ignore
  public void testRetractFact() {
  }

  @Ignore @Test
  public void testcreateDatabaseFromFile() {
    Database.getInstance().loadDatabaseFromFile("edu/tufts/hrilab/action/db/scripts/actioncore.xml");
    ActionDBEntry liftup = Database.getActionDB().getAction("liftup");
    assertNotNull(liftup);
  }

  @Test
  public void testLoadSavePerformanceMeasuresFromFile() {

    //Path to resources on classpath. Resources can only be loaded, not saved.
    String perfResourceDir = "config/edu/tufts/hrilab/action/performancemodels";
    //Path to directory to save runtime files. Java resource paths cannot be used to save files.
    String perfFileDir = "src/test/resources/config/edu/tufts/hrilab/action/performancemodels/";

    ActionDBEntry.Builder goToLocation = new ActionDBEntry.Builder("goToLocation");
    goToLocation.addRole(new ActionBinding.Builder("?location",edu.tufts.hrilab.fol.Symbol.class).build());
    goToLocation.addEffect(new Effect(Factory.createPredicate("at(?actor,?location)"), EffectType.SUCCESS));
    ActionDBEntry gtlDBEntry = goToLocation.build(true);

    ActionDBEntry.Builder openGripper = new ActionDBEntry.Builder("openGripper");
    openGripper.addRole(new ActionBinding.Builder("?groupName",edu.tufts.hrilab.fol.Symbol.class).build());
    ActionDBEntry ogDBEntry = openGripper.build(true);

    ActionDBEntry.Builder look = new ActionDBEntry.Builder("look");
    look.addRole(new ActionBinding.Builder("?direction",edu.tufts.hrilab.fol.Symbol.class).build());
    ActionDBEntry lDBEntry = look.build(true);

    Database.getInstance().loadPerformanceMeasuresFromFile(perfResourceDir, Collections.singletonList("test_data.json"));

    PerformanceMeasures gtlPM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(gtlDBEntry);
    PerformanceMeasures ogPM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(ogDBEntry);
    PerformanceMeasures lPM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(lDBEntry);

    assertNotNull(gtlPM);
    assertNotNull(ogPM);
    assertNotNull(lPM);

    String fn = "saved_test_data.json";
    File savedFile = new File(perfFileDir + fn);
    if (savedFile.exists()) {
      savedFile.delete();
    }

    Database.getPerformanceMeasuresDB().savePerformanceMeasures(perfFileDir, fn);
    try {
      String test_string = new BufferedReader(new FileReader(perfFileDir + "test_data.json")).lines().collect(Collectors.joining());
      String saved_string = new BufferedReader(new FileReader(perfFileDir + fn)).lines().collect(Collectors.joining());

      JSONAssert.assertEquals(test_string, saved_string, JSONCompareMode.NON_EXTENSIBLE);
    } catch (FileNotFoundException | JSONException e){
      fail();
    } finally {
      // delete tmp file
      if (savedFile.exists()) {
        savedFile.delete();
      }
    }
  }

  List<ActionBinding> createActionBindings(int index) {
    List<ActionBinding> bindings = new ArrayList<>();
    bindings.add(new ActionBinding.Builder("?actor", String.class).setValue(Factory.createSymbol("fluffy:agent")).build());
    bindings.add(new ActionBinding.Builder("?object", String.class).setValue("ball").build());
    if (index ==0) {
      bindings.add(new ActionBinding.Builder("?arg2", Object.class).setValue("here").build());
    } else if (index == 1) {
      bindings.add(new ActionBinding.Builder("?arg2", Object.class).setValue("there").build());
    }
    return bindings;
  }


  private void dummy1() {}
  public void dummy2() {}
  private static void dummy3() {}
  public static void dummy4() {}

  @OperatorSymbol("555")
  public static void dummy5() {}

  public static void dummy6(int a) {}
  public static void dummy6(int a, int b) {}
  public static void dummy6(int a, float b) {}

  @Test
  public void testAddOperator() {
    try {
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy1"));
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy2"));
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy3"));
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy4"));
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy5"));
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy6", int.class));
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy6", int.class, int.class));
      Database.getOperatorDB().addOperator(DatabaseTest.class.getDeclaredMethod("dummy6", int.class, float.class));
    }
    catch (NoSuchMethodException e) {
      System.out.println("Failed to get dummy methods! " + e);
    }

    assertNull(Database.getOperatorDB().getOperator("dummy1"));
    assertNull(Database.getOperatorDB().getOperator("dummy2"));
    assertNull(Database.getOperatorDB().getOperator("dummy3"));
    assertNotNull(Database.getOperatorDB().getOperator("dummy4"));
    assertNull(Database.getOperatorDB().getOperator("dummy5"));
    assertNotNull(Database.getOperatorDB().getOperator("555"));

    ArrayList<Class<?>> roles = new ArrayList<>();
    roles.add(int.class);
    OperatorDBEntry op1 = Database.getOperatorDB().getOperator("dummy6", roles);
    roles.add(int.class);
    OperatorDBEntry op2 = Database.getOperatorDB().getOperator("dummy6", roles);
    roles.remove(1);
    roles.add(float.class);
    OperatorDBEntry op3 = Database.getOperatorDB().getOperator("dummy6", roles);

    assertNotNull(op1);
    assertNotNull(op2);
    assertNotNull(op3);
    assertNotSame(op1, op2);
    assertNotSame(op2, op3);
    assertNotSame(op3, op1);
  }
}
