/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db.performanceMeasures.GSONObjects;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ConfigDependentMeasures {
  private String group;
  private int[] jointDistribution;

  public String getGroup() {
    return group;
  }

  public List<Integer> getJointDistribution() {
    ArrayList<Integer> jDistribution = new ArrayList<>(jointDistribution.length);
    if (jointDistribution.length > 0) {
      Arrays.stream(jointDistribution).forEach(x -> jDistribution.add(x));
    }
    return jDistribution;
  }

  public ConfigDependentMeasures(String groupName, int[] distribution) {
    group = groupName;
    jointDistribution = distribution;
  }
}
