/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.util.HashMap;
import java.util.Map;

public class Hypothesis {
  private final Map<Variable, Symbol> assignments;
  private final double likelihood;

  public Hypothesis(Map<Variable, Symbol> assignments, double likelihood) {
    this.assignments = assignments;
    this.likelihood = likelihood;
  }

  public Map<Variable, Symbol> assignments() {
    return assignments;
  }

  public double likelihood() {
    return likelihood;
  }

  public Hypothesis merge(Hypothesis other) {
    Map<Variable, Symbol> merged = new HashMap<>(this.assignments);
    merged.putAll(other.assignments);
    return new Hypothesis(merged, this.likelihood * other.likelihood);
  }

  @Override
  public String toString() {
    return "Hypothesis(" + assignments + ", " + likelihood + ")";
  }
}
