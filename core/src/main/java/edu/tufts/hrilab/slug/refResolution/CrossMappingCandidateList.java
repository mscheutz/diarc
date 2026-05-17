/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import java.util.List;

/**
 * The result of combining together multiple single-variable candidate lists: a set of candidate
 * variable-reference bindings, and an aggregate probability for that joint mapping.
 */
public class CrossMappingCandidateList {
  private final List<RelevanceTheoreticBinding> bindings;
  private final double probability;

  public CrossMappingCandidateList(List<RelevanceTheoreticBinding> bindings, double probability) {
    this.bindings = bindings;
    this.probability = probability;
  }

  public List<RelevanceTheoreticBinding> bindings() {
    return bindings;
  }

  public double probability() {
    return probability;
  }

  @Override
  public String toString() {
    return "CrossMappingCandidateList(" + bindings + ", " + probability + ")";
  }
}
