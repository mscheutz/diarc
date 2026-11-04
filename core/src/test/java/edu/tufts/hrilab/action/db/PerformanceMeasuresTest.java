/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.db.performanceMeasures.PerformanceMeasures;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.json.JSONException;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.skyscreamer.jsonassert.JSONAssert;
import org.skyscreamer.jsonassert.JSONCompareMode;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;


public class PerformanceMeasuresTest {
  private static PerformanceMeasures goToLocationPM, openGripperPM, pickupPM, lookPM, placePM;
  private static List<ActionBinding> actionBindings;
  private final static int randomNumberSeed = 10;
  private static Predicate graspingPred = Factory.createPredicate("grasping(?actor,?object,?arm)");
  private static Predicate touchingPred = Factory.createPredicate("touching(?actor,?object,?arm)");

  /**
   * Path to resources on classpath. Resources can only be loaded, not saved.
   */
  private static String perfResourceDir = "config/edu/tufts/hrilab/action/performancemodels";
  /**
   * Path to directory to save runtime files. Java resource paths cannot be used to save files.
   */
  private static String perfFileDir = "src/test/resources/config/edu/tufts/hrilab/action/performancemodels/";

  @BeforeClass
  public static void setupPerformanceMeasures() {
    ActionDBEntry.Builder goToLocation = new ActionDBEntry.Builder("goToLocation");
    goToLocation.addRole(new ActionBinding.Builder("?location",edu.tufts.hrilab.fol.Symbol.class).build());
    goToLocation.addEffect(new Effect(edu.tufts.hrilab.fol.Factory.createPredicate("at(?actor,?location)"), edu.tufts.hrilab.action.EffectType.SUCCESS));
    ActionDBEntry goToLocationDBE = goToLocation.build(true);

    ActionDBEntry.Builder openGripper = new ActionDBEntry.Builder("openGripper");
    openGripper.addRole(new ActionBinding.Builder("?arm",edu.tufts.hrilab.fol.Symbol.class).build());
    ActionDBEntry openGripperDBE = openGripper.build(true);

    ActionDBEntry.Builder pickup = new ActionDBEntry.Builder("pickup");
    pickup.addRole(new ActionBinding.Builder("?object", edu.tufts.hrilab.fol.Symbol.class).build());
    pickup.addRole(new ActionBinding.Builder("?arm", edu.tufts.hrilab.fol.Symbol.class).build());
    //pickup.addEffect(new Effect(Factory.createPredicate("holding(?actor,?object)"), EffectType.SUCCESS));
    pickup.addEffect(new Effect(graspingPred, edu.tufts.hrilab.action.EffectType.SUCCESS));
    pickup.addEffect(new Effect(touchingPred, edu.tufts.hrilab.action.EffectType.SUCCESS));
    ActionDBEntry pickupDBE = pickup.build(true);

    ActionDBEntry.Builder place = new ActionDBEntry.Builder("place");
    place.addRole(new ActionBinding.Builder("?object", edu.tufts.hrilab.fol.Symbol.class).build());
    place.addRole(new ActionBinding.Builder("?arm", edu.tufts.hrilab.fol.Symbol.class).build());
    place.addEffect(new Effect(graspingPred.toNegatedForm(), edu.tufts.hrilab.action.EffectType.SUCCESS));
    place.addEffect(new Effect(touchingPred.toNegatedForm(), edu.tufts.hrilab.action.EffectType.SUCCESS));
    ActionDBEntry placeDBE = place.build(true);

    ActionDBEntry.Builder look = new ActionDBEntry.Builder("look");
    look.addRole(new ActionBinding.Builder("?direction",edu.tufts.hrilab.fol.Symbol.class).build());
    ActionDBEntry lookDBE = look.build(true);

    Database.getInstance().loadPerformanceMeasuresFromFile(perfResourceDir, Collections.singletonList("test_data.json"));

    ActionBinding actor = new ActionBinding.Builder("?actor", edu.tufts.hrilab.fol.Symbol.class).setValue("andy").build();
    ActionBinding location = new ActionBinding.Builder("?location", edu.tufts.hrilab.fol.Symbol.class).setValue("woods").build();
    ActionBinding arm = new ActionBinding.Builder("?arm", edu.tufts.hrilab.fol.Symbol.class).setValue("arm").build();
    ActionBinding object = new ActionBinding.Builder("?object", edu.tufts.hrilab.fol.Symbol.class).setValue("rock").build();
    ActionBinding direction = new ActionBinding.Builder("?direction", edu.tufts.hrilab.fol.Symbol.class).setValue("up").build();

    actionBindings = Arrays.asList(location, arm, object, direction, actor);

    goToLocationPM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(goToLocationDBE);
    openGripperPM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(openGripperDBE);
    pickupPM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(pickupDBE);
    lookPM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(lookDBE);
    placePM = Database.getPerformanceMeasuresDB().getPerformanceMeasures(placeDBE);

  }
  @Before
  public void setRandomSeed() {
    PerformanceMeasures.setSeed(randomNumberSeed);
  }

  @Test
  public void sampleEffectsTest() {
    Map<Predicate,Boolean> sampledValues = pickupPM.sampleEffects(actionBindings);
    assertTrue(sampledValues.get(Factory.createPredicate("touching(andy,rock,arm)")));
    assertTrue(sampledValues.get(Factory.createPredicate("grasping(andy,rock,arm)")));
    int numSuccess = 0;
    double totalNum = 1000;
    for (int i=0; i<totalNum; i++) {
      sampledValues = pickupPM.sampleEffects(actionBindings);
      if(sampledValues.entrySet().stream().allMatch(Map.Entry::getValue)) {
        numSuccess ++;
      }
    }
    // probability of all effects holding for pickup = 16/20
    // numSuccess should = 820
    assertTrue(Math.abs(16/20.0 - numSuccess/totalNum) <= (1/20.0));

    numSuccess = 0;
    for (int i=0; i<totalNum; i++) {
      sampledValues = goToLocationPM.sampleEffects(actionBindings);
      if (sampledValues.entrySet().stream().allMatch(Map.Entry::getValue)) {
        numSuccess++;
      }
    }
    // probability effects hold for go to location = 14/20
    // numSuccess should = 724
    assertTrue(Math.abs(14/20.0 - numSuccess/totalNum) <= (1/20.0));
  }

  @Test
  public void sampleSuccessTest() {
    assertFalse(lookPM.sampleSuccess(actionBindings));
    assertTrue(openGripperPM.sampleSuccess(actionBindings));
    int numSuccess = 0;
    double totalNum = 1000;
    for (int i=0; i<totalNum; i++) {
      if(lookPM.sampleSuccess(actionBindings)) {
        numSuccess ++;
      }
    }
    // probability of all effects holding for look = 100/150
    // numSuccess should = 662
    assertTrue(Math.abs(100/150.0 - numSuccess/totalNum) <= (0.05));

    numSuccess = 0;
    for (int i=0; i<totalNum; i++) {
      if (openGripperPM.sampleSuccess(actionBindings)) {
        numSuccess++;
      }
    }
    // probability effects hold for openGripper = 93/100
    // numSuccess should = 925
    assertTrue(Math.abs(93/100.0 - numSuccess/totalNum) <= (0.05));
  }

  @Test
  public void sampleTimeTest() {
    double time = goToLocationPM.sampleTime(actionBindings, true);
    System.out.println(time);
    time = goToLocationPM.sampleTime(actionBindings, false);
    System.out.println(time);
    time = openGripperPM.sampleTime(actionBindings, true);
    System.out.println(time);
    time = openGripperPM.sampleTime(actionBindings, false);
    System.out.println(time);

  }


  @Test
  public void updatePerformanceModelsTest() {
    ActionBinding actor = new ActionBinding.Builder("?actor", edu.tufts.hrilab.fol.Symbol.class).setValue("andy").build();
    ActionBinding arm = new ActionBinding.Builder("?arm", edu.tufts.hrilab.fol.Symbol.class).setValue("arm").build();
    ActionBinding object = new ActionBinding.Builder("?object", edu.tufts.hrilab.fol.Symbol.class).setValue("rock").build();
    List<ActionBinding> actBindings = Arrays.asList(actor, object, arm);
    Map<Predicate, Boolean> effectsHold = new HashMap<>();

    effectsHold.put(graspingPred.toNegatedForm(), false);
    effectsHold.put(touchingPred.toNegatedForm(), true);

    placePM.updatePerformanceModels(false, 15000, effectsHold, actBindings);

    effectsHold.put(graspingPred.toNegatedForm(), true);
    effectsHold.put(touchingPred.toNegatedForm(), true);

    placePM.updatePerformanceModels(true, 17000, effectsHold, actBindings);

    effectsHold.put(graspingPred.toNegatedForm(), false);
    effectsHold.put(touchingPred.toNegatedForm(), false);

    placePM.updatePerformanceModels(false, 13000, effectsHold, actBindings);

    String fn = "resulting_updated_models_test_data.json";
    File savedFile = new File(perfFileDir + fn);
    if (savedFile.exists()) {
      savedFile.delete();
    }

    Database.getPerformanceMeasuresDB().savePerformanceMeasures(perfFileDir, fn);
    try {
      String test_string = new BufferedReader(new FileReader(perfFileDir + "update_models_test_data.json")).lines().collect(Collectors.joining());
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
}