/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;

import java.util.ArrayList;
import java.util.List;

public class ConfigEffectDependent {
  String group;
  String[] effects;

  public ConfigEffectDependent(String groupName, List<Predicate> effects) {
    group = groupName;
    this.effects = new String[effects.size()];
    for (int i = 0; i < effects.size(); i++) {
      this.effects[i] = effects.get(i).toString();
    }
    for (Predicate eff : effects) {

    }
  }

  public ConfigEffectDependent() {
    effects = new String[0];
  }



  public String getGroup() {
    return group;
  }

  public String[] getEffects() {
    return effects;
  }
}
