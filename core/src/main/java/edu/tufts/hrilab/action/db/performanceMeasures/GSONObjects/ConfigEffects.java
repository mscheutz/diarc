/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects;

import edu.tufts.hrilab.fol.Predicate;

import java.util.ArrayList;
import java.util.List;

public class ConfigEffects {
  ConfigEffectDependent[] dependent;
  String[] independent;

  public ConfigEffects(List<Predicate> independentEffects, List<ConfigEffectDependent> dependentEffects) {
    if (!independentEffects.isEmpty()) {
      independent = new String[independentEffects.size()];
      for (int i=0; i < independentEffects.size(); i++) {
        independent[i] = independentEffects.get(i).toString();
      }
    } else {
      independent = new String[0];
    }
    if (!dependentEffects.isEmpty()) {
      dependent = new ConfigEffectDependent[dependentEffects.size()];
      for (int i = 0; i < dependentEffects.size(); i++) {
        dependent[i] = dependentEffects.get(i);
      }
    } else {
      dependent = new ConfigEffectDependent[0];
    }
  }

  public String[] getIndependentEffects() {
    return independent;
  }

  public ConfigEffectDependent[] getDependentEffects() {
    //return null;
    return dependent;
  }
}
