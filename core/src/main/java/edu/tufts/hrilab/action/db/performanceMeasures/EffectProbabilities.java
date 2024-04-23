/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigEffectMeasures;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigDependentMeasures;
import edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects.ConfigMarginal;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class EffectProbabilities {
  private static Logger log = LoggerFactory.getLogger(EffectProbabilities.class);
  private int experience;
  private ArrayList<Predicate> independentEffects;
  private Map<Predicate,Integer> marginalProbs;
  private Map<String, List<Integer>> jointDists;
  private Map<String, List<Predicate>> dependentGroups;
  private Map<Predicate, String> effectDependentGroup;
  private Random rng;
  int seed = 30;

  public EffectProbabilities() {
    independentEffects = new ArrayList<>();
    marginalProbs = new HashMap<>();
    jointDists = new HashMap<>();
    dependentGroups = new HashMap<>();
    effectDependentGroup = new HashMap<>();
    rng = new Random(seed);
    experience = 0;
  }

  /**
   * set all the effects for new action without configuration
   * @param effects
   */
  public void setEffectInformation(List<Predicate> effects) {
    if (effects.isEmpty()) {
      return;
    }
    String groupName = "fj";
    dependentGroups.put(groupName, effects);
    for (Predicate effect : effects) {
      effectDependentGroup.put(effect, groupName);
    }
    ArrayList<Integer> jointDist = new ArrayList<>(Arrays.asList(new Integer[(int)Math.pow(2,effects.size())]));
    Collections.fill(jointDist,0);
    jointDists.put(groupName, jointDist);
  }

  public void setEffectInformation(Map<String, List<Predicate>> dependentGroups) {
    for (Map.Entry<String, List<Predicate>> group : dependentGroups.entrySet()) {
      if (!this.dependentGroups.containsKey(group.getKey())) {
        this.dependentGroups.put(group.getKey(), group.getValue());
      } else {
        // override full joint dist from initial setup with config
        if (group.getKey().equals("fj")) {
          this.dependentGroups.put("fj", group.getValue());
        }
      }
      for (Predicate effect : group.getValue()) {
        if (!effectDependentGroup.containsKey(effect)) {
          effectDependentGroup.put(effect,group.getKey());
        } else { // need to override effectDependency from initial setup
          if (effectDependentGroup.get(effect).equals("fj")) {
            effectDependentGroup.put(effect,group.getKey());
          }
        }
      }
    }
  }

  // TODO: change this so it properly sets the effect info to take into account effect dependency
  protected List<Predicate> getEffects() {
    if (dependentGroups.isEmpty()) {
      return new ArrayList<>();
    }
    return dependentGroups.get("fj");
  }

//////////////////////////////////////////////////
// Update Models
  /**
   * incrementally update the effect probabilities
   * @param effectHolds map of whether each effect holds
   */
  public void updateModels(Map<Predicate, Boolean> effectHolds) {
    // update marginal probability
    experience += 1;
    for (Map.Entry<Predicate, Boolean> entry : effectHolds.entrySet()) {
      Predicate effect = entry.getKey();
      if (!(effect.getName().equals("succeeded") && effect.getName().equals("failed"))) {
        if (entry.getValue()) {
          int updated = 1;
          if (marginalProbs.containsKey(effect)) {
            updated = marginalProbs.get(effect) + 1;
          }
          marginalProbs.put(effect, updated);
        }
      }
    }
    // update joint distributions
    for (Map.Entry<String, List<Predicate>> group : dependentGroups.entrySet()) {
      String groupName = group.getKey();
      List<Predicate> effects = group.getValue();
      if (!effects.isEmpty()) {
        int location = 0;
        for (int i = 0; i < effects.size(); i++) {
          int val = 0;
          if (effectHolds.containsKey(effects.get(i))) {
            if (effectHolds.get(effects.get(i))) {
              val = 1;
            }
            location += val * Math.pow(2, i);
          } else {
            log.debug("Effect " + effects.get(i) + " was not provided. skipping remaining effects");
            break;
          }
        }
        List<Integer> jointDist = jointDists.get(groupName);
        int updatedVal = jointDist.get(location) + 1;
        jointDist.set(location, updatedVal);
      }
    }
  }

  /**
   * batch update effect probabilities
   * @param newMarginalProbs map from effect to number of times effect holds
   * @param jointDistribution map from group of dependent effects to joint distribution
   */
  public void updateModels(Map<Predicate,Integer> newMarginalProbs, Map<String, List<Integer>> jointDistribution, Map<String, List<Predicate>> dependentGroups, int experience) {
    // update marginal probs
    this.experience += experience;
    for (Map.Entry<Predicate,Integer> marginalP : newMarginalProbs.entrySet()) {
      Predicate p = marginalP.getKey();
      int updatedMarg = marginalP.getValue();
      if (marginalProbs.containsKey(p)) {
        updatedMarg += marginalProbs.get(p);
      }
      marginalProbs.put(p,updatedMarg);
    }

    setEffectInformation(dependentGroups);

    // update joint distributions
    for (Map.Entry<String, List<Integer>> group : jointDistribution.entrySet()) {
      String groupName = group.getKey();
      List<Integer> inputDist = group.getValue();
      ArrayList<Integer> outDist = new ArrayList<>(Arrays.asList(new Integer[inputDist.size()]));
      Collections.fill(outDist,0);
      if (jointDists.containsKey(groupName)) {
        outDist = new ArrayList<>(jointDists.get(groupName));
      }

      // loop through new distribution and update joint distribution
      for (int i=0; i < inputDist.size(); i++) {
        int tmpVal = outDist.get(i);
        outDist.set(i, tmpVal + inputDist.get(i));
      }

      jointDists.put(groupName, outDist);
    }
  }

//////////////////////////////////////////////////
// Sample effects
//////////////////////////////////////////////////

  public Map<Predicate, Boolean> sampleEffects() {
    return sampleEffects(new HashMap<>());
  }

  public Map<Predicate, Boolean> sampleEffects(Map<Predicate, Boolean> given) {
    Map<Predicate, Boolean> sampledEffects = new HashMap<>();
    if (experience > 0) {
      for (Map.Entry<String, List<Predicate>> group : dependentGroups.entrySet()) {
        sampledEffects.putAll(sampleDependentEffects(group.getKey(), given));
      }
      return sampledEffects;
    }
    for (Map.Entry<String, List<Predicate>> group : dependentGroups.entrySet()) {
      List<Predicate> effects = new ArrayList<>(dependentGroups.get(group.getKey()));
      for (Predicate effect : effects) {
          sampledEffects.put(effect, true);
      }
    }
    return sampledEffects;
  }

  //private Map<Predicate, Boolean> sampleIndependentEffects() {
  //  HashMap<Predicate, Boolean> sampledEffects = new HashMap<>();
  //  for (Predicate eff : independentEffects) {
  //    double prob = marginalProbs.get(eff);
  //    sampledEffects.put(eff, rng.nextDouble() < prob);
  //  }
  //  return sampledEffects;
  //}

  // The effects are sampled based on their list index and given values are used when calculating marginal probs
  private Map<Predicate, Boolean> sampleDependentEffects(String groupName, Map<Predicate, Boolean> given) {
    HashMap<Predicate, Boolean> sampledEffects = new HashMap<>(given);
    List<Predicate> effects = new ArrayList<>(dependentGroups.get(groupName));
    Collections.shuffle(effects, rng);
    for (Predicate effect : effects) {
      if (sampledEffects.isEmpty()) {
        double prob = marginalProbs.get(effect) / (double) experience;
        boolean holds = rng.nextDouble() < prob;
        sampledEffects.put(effect, holds);
      } else {
        if (!sampledEffects.containsKey(effect)) { // if effect not already provided, then calculate its marginal prob, then sample
          Pair<Integer,Integer> marginal = getMarginal(effect, sampledEffects, groupName);
          double prob = marginal.getLeft() / (double) marginal.getRight();
          boolean holds = rng.nextDouble() < prob;
          sampledEffects.put(effect, holds);
        }
      }
    }
    return sampledEffects;
  }

  //// sample single effect, provided some given state
  //public boolean sampleEffect(Predicate effect, Map<Predicate, Boolean> given) {
  //  if (independentEffects.contains(effect)) {
  //    double probability = marginalProbs.get(effect);
  //    double rn = rng.nextDouble();
  //    return rn < probability;
  //  } else if (effectDependentGroup.containsKey(effect)){
  //    String groupName = effectDependentGroup.get(effect);
  //    return sampleEffect(groupName, effect, given);
  //  }
  //  return true;
  //}

  // sample effect from dependent group
  private boolean sampleEffect(String groupName, Predicate effect, Map<Predicate, Boolean> given) {
    List<Predicate> groupEffects = dependentGroups.get(groupName);
    Map<Predicate, Boolean> tmpGiven = new HashMap<>(given);
    Pair<Integer, Integer> marginal = getMarginal(effect, tmpGiven, groupName);
    int total = marginal.getRight();
    //tmpGiven.put(effect,true);
    //marginal = getMarginal(groupEffects.get(0), tmpGiven, groupEffects, groupName);
    //total += marginal.getRight();
    double marginalProb = marginal.getLeft() / (double)total;
    return  rng.nextDouble() < marginalProb;
    /*
      total = getMarginal(given + (effect, false)
      marginal = getMarginal(given + (effect,true)
      total += marginal
     */
  }

  private Pair<Integer, Integer> getMarginal(Predicate effect, Map<Predicate, Boolean> given, String groupName) {
    // TODO: need to take into account the effects which are indexed before 'effect'
    //       need to basically start from the root and work way down.
    //       when starting from 0 index
    //         if effect @ index ! given
    //           need to sum over both values
    given.put(effect,false); // get total number of values when effect is false
    // need to calculate because there could be given effect values which would create subset of total
    int total = getTotal(dependentGroups.get(groupName).get(0), given, 0, groupName);
    given.put(effect,true); // get total number when effect is true
    //int marginal = getTotal(groupEffects.get(0), given, (int)Math.pow(2,index), groupEffects, groupName);
    int marginal = getTotal(dependentGroups.get(groupName).get(0), given, 0, groupName);
    total += marginal;
    return Pair.of(marginal,total);
  }

  // gets total count for given branch. sum over T/F if not already provided
  private int getTotal(Predicate effect, Map<Predicate, Boolean> given, int location, String groupName) {
    int total = 0;
    List<Predicate> groupEffects = dependentGroups.get(groupName);
    int index = groupEffects.indexOf(effect);
    List<Integer> jointD = jointDists.get(groupName);
    if (given.containsKey(effect)) { // if in given, then need to get total based on value
      boolean holds = given.get(effect); // get value of effect
      int newLocation = location;
      if (holds) {
        newLocation = location + (int)Math.pow(2, index); // if holds then get associated binary location
      }
      if (index == (groupEffects.size() - 1)) { // if it is the last effect, get the value
        total = jointD.get(newLocation);
      } else { // otherwise, total based on given value
        total = getTotal(groupEffects.get(index + 1), given, newLocation, groupName);
      }
    } else { // effect has not been given, thus need to get total for both false and true branch
      if (index == (groupEffects.size() - 1)) { // if it is the last effect, get summation of both branches
        total = jointD.get(location);
        total += jointD.get(location + (int) Math.pow(2, index));
      } else { // continue on to get summation of both branches
        total = getTotal(groupEffects.get(index + 1), given, location, groupName);
        total += getTotal(groupEffects.get(index + 1), given, location + (int) Math.pow(2, index), groupName);
      }
    }
    return total;
  }


  /**
   * create GSON object to save Models
   */
  public ConfigEffectMeasures createEffectMeasuresGson() {
    ConfigMarginal[] configMarginal = new ConfigMarginal[marginalProbs.size()];
    int index = 0;
    for (Map.Entry<Predicate, Integer> marginalP : marginalProbs.entrySet()) {
      configMarginal[index++] = new ConfigMarginal(marginalP.getKey().toString(), marginalP.getValue());
    }

    ConfigDependentMeasures[] depMeasures = new ConfigDependentMeasures[jointDists.size()];
    index = 0;
    for (Map.Entry<String, List<Integer>> group : jointDists.entrySet()) {
      int[] distArray = group.getValue().stream().mapToInt(x->x).toArray();
      depMeasures[index++] = new ConfigDependentMeasures(group.getKey(), distArray);
    }

    return new ConfigEffectMeasures(configMarginal, depMeasures);
  }
}
