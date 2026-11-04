/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ConfigEffectMeasures {
  private ConfigDependentMeasures[] dependentMeasures;
  private ConfigMarginal[] marginalMeasures;

  public ConfigEffectMeasures() {
    dependentMeasures = new ConfigDependentMeasures[0];
    marginalMeasures = new ConfigMarginal[0];
  }

  public Map<String, List<Integer>> getJointDistributions() {
    HashMap<String, List<Integer>> jointDistributions = new HashMap<>();
    for (ConfigDependentMeasures dependentMeasure : dependentMeasures) {
      jointDistributions.put(dependentMeasure.getGroup(), dependentMeasure.getJointDistribution());
    }
    return jointDistributions;
  }

  public Map<Predicate, Integer> getMarginalMeasures() {
    HashMap<Predicate, Integer> marginalMap = new HashMap<>();
    for (ConfigMarginal marginal : marginalMeasures) {
      marginalMap.put(Factory.createPredicate(marginal.getName()), marginal.getHolds());
    }
    return marginalMap;
  }

  // no real reason why there are two constructors. should probably only have one
  public ConfigEffectMeasures(ConfigMarginal[] marginalProb, ConfigDependentMeasures[] dependentGroups) {
    dependentMeasures = dependentGroups;
    marginalMeasures = marginalProb;
  }

  public ConfigEffectMeasures(List<ConfigMarginal> marginalProb, List<ConfigDependentMeasures> dependentGroups) {
    marginalMeasures = new ConfigMarginal[marginalProb.size()];
    int index;
    for (index = 0; index < marginalProb.size(); index++) {
      marginalMeasures[index] = marginalProb.get(index);
    }
    dependentMeasures = new ConfigDependentMeasures[dependentGroups.size()];
    for (index = 0; index < dependentGroups.size(); index++) {
      dependentMeasures[index] = dependentGroups.get(index);
    }
  }

}
