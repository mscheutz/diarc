/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigEffectDependent;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigEffectMeasures;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigEffects;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigPerformanceMeasures;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.stream.JsonWriter;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PerformanceMeasures {
  private final static Random rng = new Random(10);
  private final static Logger log = LoggerFactory.getLogger(PerformanceMeasures.class);
  private String actionSignature;
  private List<Predicate> allEffects;
  private List<Predicate> independentEffects;
  private Map<String, List<Predicate>> dependentGroups;
  private PerformanceMeasuresNode performanceMeasuresTree;

  private int nextGroup;

  public PerformanceMeasures(ActionDBEntry action) {
    init(action);
  }

  public PerformanceMeasures(String actName) {
    init();
    actionSignature = actName;
  }

  private void init() {
    allEffects = new ArrayList<>();
    independentEffects = new ArrayList<>();
    dependentGroups = new HashMap<>();
    performanceMeasuresTree = new PerformanceMeasuresNode(Factory.createSymbol("root"));
    nextGroup=0;
    //setSeed(System.currentTimeMillis()); // real world
  }

  private void init(ActionDBEntry action) {
    init();
    List<Symbol> args = new ArrayList<>();
    for (Predicate signature : action.getSignatureOptions(true)) {
      if (signature.getArgs().size() > args.size()) {
        args = signature.getArgs();
        actionSignature = signature.toString();
      } else if (signature.getArgs().isEmpty()) {
        actionSignature = signature.toString();
      }
    }
    for (Effect effect : action.getEffects()) {
      if (!effect.getPredicate().getName().equals("succeeded") && !effect.getPredicate().getName().equals("failed")) {
        allEffects.add(effect.getPredicate());
      }
    }
    dependentGroups.put("fj", allEffects);
    performanceMeasuresTree.setEffectInformation(allEffects);
  }

  public static void setSeed(int seed) {
    rng.setSeed(seed);
  }

//////////////////////////////////////////////////
// Sample Performance Models
//////////////////////////////////////////////////

  ///**
  // * sample if the effect holds
  // * @param effect
  // * @return
  // */
  //public boolean sampleEffect(Predicate effect, List<ActionBinding> actionBindings) {
  //  return sampleEffect(effect, actionBindings, new HashMap<>());
  //}

  //public boolean sampleEffect(Predicate effect, List<ActionBinding> actionBindings, boolean isSuccess) {
  //  return true;
  //}

  //public boolean sampleEffect(Predicate effect, List<ActionBinding> actionBindings, Map<Predicate, Boolean> givenEffects) {
  //  LinkedHashMap<Symbol, Symbol> paramArgs = extractParamArgs(actionBindings);
  //  ArrayList<Symbol> argValues = new ArrayList<>();
  //  for (Map.Entry<Symbol, Symbol> entry : paramArgs.entrySet()) {
  //    argValues.add(entry.getValue());
  //  }
  //  PerformanceMeasuresNode pm = performanceMeasuresTree.getPerformanceMeasures(argValues);
  //  return pm.sampleEffect(effect, givenEffects);
  //}

  public Map<Predicate, Boolean> sampleEffects(List<ActionBinding> actionBindings, Map<Predicate, Boolean> givenEffects, boolean isSuccess) {
    Map<Predicate, Boolean> effectsHold = new HashMap<>();
    return effectsHold;
  }

  public Map<Predicate, Boolean> sampleEffects(List<ActionBinding> actionBindings, Map<Predicate, Boolean> givenEffects) {
    LinkedHashMap<Symbol, Symbol> paramArgs = extractParamArgs(actionBindings);
    ArrayList<Symbol> argValues = new ArrayList<>();
    for (Map.Entry<Symbol, Symbol> entry : paramArgs.entrySet()) {
      argValues.add(entry.getValue());
    }
    PerformanceMeasuresNode pm = performanceMeasuresTree.getPerformanceMeasures(argValues);
    /*
      need to get the probabilities of each effect, some may be dependent.
      need to know which effects are dependent
     */
    Map<Predicate, Boolean> effectHolds = pm.sampleEffects(givenEffects);
    HashMap<Predicate, Boolean> boundEffectHolds = new HashMap<>();
    for (Map.Entry<Predicate,Boolean> entry : effectHolds.entrySet()) {
      String predName = entry.getKey().getName();
      ArrayList<Symbol> args = new ArrayList<>();
      List<Symbol> entryArgs = entry.getKey().getArgs();
      boolean isNegated = entry.getKey().getName().equals("not");
      if (isNegated) {
        predName = entryArgs.get(0).getName();
        entryArgs =  ((Predicate)entryArgs.get(0)).getArgs();
      }
      for (Symbol param : entryArgs) {
        boolean foundParam = false;
        for (ActionBinding binding : actionBindings) {
          if (binding.getName().equals(param.getName())) {
            args.add(Factory.createFOL(binding.getBindingDeep().toString()));
            foundParam = true;
            break;
          }
        }
        if (!foundParam) {
          args.add(param);
        }
      }
      Predicate boundPred = Factory.createPredicate(predName, args);
      if (isNegated) {
        boundPred = Factory.createNegatedPredicate(boundPred);
      }
      boundEffectHolds.put(boundPred, entry.getValue());
    }
    return boundEffectHolds;
  }

  public Map<Predicate, Boolean> sampleEffects(List<ActionBinding> actionBindings) {
    return sampleEffects(actionBindings, new HashMap<>());
  }

  public double sampleTime(List<ActionBinding> actionBindings, boolean holds) {
    //holds = true;
    List<Symbol> argValues = extractArgumentValues(actionBindings);
    PerformanceMeasuresNode probNode = performanceMeasuresTree.getPerformanceMeasures(argValues);
    return sampleTime(probNode, holds);
  }

  private double sampleTime(PerformanceMeasuresNode probNode, boolean holds) {
    //holds = true;
    Pair<Double, Double> timeProbability = probNode.getTimeDistribution(holds);
    double sampledTime = 0;
    if (timeProbability.getLeft() == 0) {
      sampledTime = 1;
    }
    while (sampledTime <= 0) {
      sampledTime = timeProbability.getLeft();
      sampledTime = sampledTime +  rng.nextGaussian() * Math.sqrt(timeProbability.getRight());
      log.debug("[sampleTime] " + timeProbability);
    }
    return sampledTime;
  }

  public boolean sampleSuccess(List<ActionBinding> actionBindings) {
    List<Symbol> argValues = extractArgumentValues(actionBindings);
    PerformanceMeasuresNode probNode = performanceMeasuresTree.getPerformanceMeasures(argValues);
    Pair<Integer, Integer> experience = probNode.getExperience();
    if (experience.getRight() > 0) {
      double prob = experience.getLeft() / (double) experience.getRight();
      return rng.nextDouble() < prob;
    }
    // TODO: handle primitive actions with no experience
    return true;
    //return rng.nextDouble() < 0.5;
  }

  // should this be available or should the caller call sampleProb and sampleTime?
  public Pair<Boolean, Double> sampleSuccessAndTime(List<ActionBinding> actionBindings) {
    List<Symbol> argValues = extractArgumentValues(actionBindings);
    PerformanceMeasuresNode probNode = performanceMeasuresTree.getPerformanceMeasures(argValues);
    Pair<Integer, Integer> experience = probNode.getExperience();
    double prob = experience.getLeft() / (double) experience.getRight();
    boolean success = rng.nextDouble() < prob;
    double time = sampleTime(probNode, success);
    return Pair.of(success, time);
  }


//////////////////////////////////////////////////
// Update performance models
//////////////////////////////////////////////////

  /**
   * batch update performance models
   * @param configMeasures performance models
   * @param args argument values for action
   */
  public void updatePerformanceModels(ConfigPerformanceMeasures configMeasures, List<Symbol> args) {
    double experience = configMeasures.getExperience();
    double success = configMeasures.getSuccess();
    double successTimeMean = configMeasures.getSuccessTimeMean();
    double successTimeVar = configMeasures.getSuccessTimeVar();
    double successSumSqDiff = configMeasures.getSuccessSumSqDiff();
    double failureTimeMean = configMeasures.getFailureTimeMean();
    double failureTimeVar = configMeasures.getFailureTimeVar();
    double failureSumSqDiff = configMeasures.getFailureSumSqDiff();
    ConfigEffectMeasures effMeasures = configMeasures.getEffectMeasures();
    performanceMeasuresTree.updatePerformanceModels(args, experience, success, successTimeMean, successTimeVar, successSumSqDiff, failureTimeMean, failureTimeVar, failureSumSqDiff, effMeasures, dependentGroups);
  }

  /**
   * incremental update of performance models
   * @param actionBindings argument bindings for action
   * @param holds action was successful
   * @param time duration of executino
   */
  public void updatePerformanceModels(List<ActionBinding> actionBindings, boolean holds, long time) {
    List<Symbol> argValues = extractArgumentValues(actionBindings);
    double timeSecs = time / Math.pow(10,3);
    performanceMeasuresTree.updatePerformanceModels(argValues, holds, timeSecs);
  }

  /**
   * incremental update of performance models
   * @param isSuccess is the action successful
   * @param time duration of execution
   * @param effectResults map of which effect predicates hold
   * @param actionBindings argument bindings for action
   */
  public void updatePerformanceModels(boolean isSuccess, long time, Map<Predicate, Boolean> effectResults, List<ActionBinding> actionBindings) {
    log.trace("updating Performance models");
    List<Symbol> argValues = extractArgumentValues(actionBindings);
    double timeSecs = time / Math.pow(10,3);
    try {
      performanceMeasuresTree.updatePerformanceModels(argValues, isSuccess, timeSecs);
      performanceMeasuresTree.updateEffectPerformanceModels(effectResults, argValues);
    } catch (Exception e)  {
      log.error("[updatePerformanceModels] ",e);
    }
  }

//////////////////////////////////////////////////
// Extract the argument values
//////////////////////////////////////////////////

  // get the argument vale for each action binding, will try to replace any arg references
  private List<Symbol> extractArgumentValues(List<ActionBinding> actionBindings) {
    log.trace("extracting argument values");
    List<Symbol> argValues =  new ArrayList<>();
    for (ActionBinding actionBinding : actionBindings) {
      if (actionBinding.isLocal() || actionBinding.isReturn()) {
        continue;
      }
      Symbol updatedArgValue = updateReference(actionBinding);
      if (updatedArgValue != null) {
        argValues.add(updatedArgValue);
      }
    }
    return argValues;
  }

  private LinkedHashMap<Symbol, Symbol> extractParamArgs(List<ActionBinding> actionBindings) {
    LinkedHashMap<Symbol, Symbol> argValues =  new LinkedHashMap<>();
    for (ActionBinding actionBinding : actionBindings) {
      Symbol updatedArgValue = updateReference(actionBinding);
      if (updatedArgValue != null) {
        argValues.put(Factory.createFOL(actionBinding.getName()), updatedArgValue);
      }
    }
    return argValues;
  }

  private List<Symbol> extractArgumentValues(List<Symbol> parameters, List<ActionBinding> actionBindings) {
    List<Symbol> args = new ArrayList<>();
    HashMap<String, ActionBinding> bindingNameMap = new HashMap<>(); // create map to easily search matching param name
    for (ActionBinding actionBinding : actionBindings) {
      bindingNameMap.put(actionBinding.getName(), actionBinding);
    }
    for (Symbol param : parameters) { // bind effect parameters in order
      ActionBinding actionBinding = bindingNameMap.get(param.getName());
      if (actionBinding != null) {
        args.add(updateReference(actionBinding));
      }
    }
    return args;
  }

  public List<Predicate> getAllEffects() {
    return allEffects;
  }

  private void setEffectInformation(ConfigEffects effectInfo) {
    String[] indEffects = effectInfo.getIndependentEffects();
    for (String effect : indEffects) {
      Predicate eff = Factory.createPredicate(effect);
      if (!independentEffects.contains(eff)) {
        independentEffects.add(eff);
      }
      if (!allEffects.contains(eff)) {
        allEffects.add(eff);
      }
    }

    ConfigEffectDependent[] dependentEffects = effectInfo.getDependentEffects();
    for (ConfigEffectDependent effect : dependentEffects) {
      setDependentGroups(effect.getEffects(), effect.getGroup());
    }
  }

  private void setDependentGroups(String[] dependentEffects, String groupName) {
    if (dependentEffects.length == 0) {
      return;
    }
    if (dependentGroups.isEmpty()) {
      createDependentGroup(dependentEffects, groupName);
    } else {
      boolean containsGroup = false;
      boolean inGroup = true;
      for (Map.Entry<String, List<Predicate>> group : dependentGroups.entrySet()) {
        List<Predicate> dependent = group.getValue();
        if (dependentEffects.length == dependent.size()) {
          for (String eff : dependentEffects) {
            Predicate e = Factory.createPredicate(eff);
            if (!dependent.contains(e)) {
               inGroup = false;
              break;
            }
          }
          if (inGroup) {
            containsGroup = true;
            break;
          }
        }
      }
      if (!containsGroup) {
        createDependentGroup(dependentEffects, groupName);
      }
    }
  }

  private void createDependentGroup(String[] dependentEffects, String groupName) {
    if (groupName == null) {
      do {
        groupName = "d" + nextGroup++;
      } while (dependentGroups.containsKey(groupName));
    }
    List<Predicate> effects = new ArrayList<>();
    for (String depEffect : dependentEffects) {
      Predicate eff = Factory.createPredicate(depEffect);
      if (!effects.contains(eff)) {
        effects.add(eff);
      }
      if (!allEffects.contains(eff)) {
        allEffects.add(eff);
      }
    }
    dependentGroups.put(groupName, effects);
  }

  private void parseMeasures(JsonObject argsObj, List<Symbol> args, Gson gson) {
    for (Map.Entry<String, JsonElement> jsonArgs : argsObj.entrySet()) { // loop through arg permutations
      ArrayList<Symbol> argValues = new ArrayList<>(args);
      if (jsonArgs.getKey().equalsIgnoreCase("performanceMeasures")) { // acquire the
        ConfigPerformanceMeasures configPerformanceMeasures = gson.fromJson(jsonArgs.getValue(), ConfigPerformanceMeasures.class);
        updatePerformanceModels(configPerformanceMeasures, argValues);
      } else {
        argValues.add(Factory.createFOL(jsonArgs.getKey()));
        parseMeasures(jsonArgs.getValue().getAsJsonObject(), argValues, gson);
      }
    }
  }

  public void populatePerformanceMeasures(JsonObject jsonMeasures) {
    Gson gson = new Gson();
    if (jsonMeasures.has("effects")) { //
      JsonElement effectsElem = jsonMeasures.get("effects");
      ConfigEffects effects = gson.fromJson(effectsElem, ConfigEffects.class);
      if (effects.getDependentEffects()[0].getEffects().length != allEffects.size()){
        return;
      }
      setEffectInformation(effects);
    }
    if (jsonMeasures.has("measures")) { // loop through and get prob for all args permutation
      JsonObject args = jsonMeasures.getAsJsonObject("measures");
      parseMeasures(args, new ArrayList<Symbol>(), gson);
    }
  }

  public boolean appendPerformanceMeasuresToJsonWriter(JsonWriter jsonWriter) {
    try {
      if (performanceMeasuresTree.getExperience().getRight() > 0) {
        jsonWriter.setIndent("  ");
        Gson gson = new Gson();
        jsonWriter.name(actionSignature);
        jsonWriter.beginObject(); // create action value obj
        jsonWriter.name("effects");
        ConfigEffects configEffects = generateEffectGsonObject();
        gson.toJson(configEffects, ConfigEffects.class, jsonWriter);
        jsonWriter.name("measures");
        jsonWriter.beginObject(); // create measures value obj
        performanceMeasuresTree.savePerformanceMeasures(jsonWriter, gson);
        jsonWriter.endObject(); // end measures value obj
        jsonWriter.endObject(); // end action value obj
      } else {
        return false;
      }
    } catch (IOException ex) {
      log.warn("unable to save performance measure " + actionSignature, ex);
      return false;
    }
    return true;
  }

  private ConfigEffects generateEffectGsonObject() {
    List<ConfigEffectDependent> configEffectDependents = new ArrayList<>();
    for (Map.Entry<String, List<Predicate>> dependentGroup : dependentGroups.entrySet()) {
      configEffectDependents.add(new ConfigEffectDependent(dependentGroup.getKey(), dependentGroup.getValue()));
    }
    ConfigEffects effects = new ConfigEffects(independentEffects, configEffectDependents);
    return effects;
  }

//////////////////////////////////////////////////
//  extra methods
//////////////////////////////////////////////////

// method to replace argument reference with argument type
// hack around no clear/easy way to dereference arg
// only checks for specific arg names, which isn't ideal
private Symbol updateReference(ActionBinding actionBinding) {
  // TODO: handle nulls properly and continue extracting argValues
  log.trace("replacing reference with argument for " + actionBinding.getName());
  Symbol argValue = null;
  if (!actionBinding.isReturn()) {
    argValue = Utilities.createFOL(actionBinding);
    if (argValue != null) {
      Symbol tmpVal = resolveReference(argValue);
      if (tmpVal != null) {
        argValue = tmpVal;
      }
    } else {
      argValue = Factory.createFOL("null");
    }
  }
  return argValue;
}

  public Symbol resolveReference(Symbol reference) {

    //TODO:brad:why is this called on everything? it should only be called on actually references...
    if(!reference.getName().contains("_")){
      return reference;
    }

    // FIXME: extremely hacky
    //        using this method to get the object type for estimating performance
    //        figure out how to only extract 'important' / relevant properties
    List<String> propertyTypesIgnore = Arrays.asList("grasp_point","on");
    try {
          List<Term> properties = TRADE.getAvailableService(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class)).call(List.class, reference);
          for (Term property : properties) {
            String propertyName = property.getName();
            if (!propertyTypesIgnore.contains(propertyName)) {
              // return the first property not in ignored property types
              // TODO: handle multiple properties
              return Factory.createFOL(propertyName);
            }
          }
    } catch (TRADEException e) {
      log.debug("couldn't resolve reference for: " + reference, e);
    }
    return null;
  }
}
