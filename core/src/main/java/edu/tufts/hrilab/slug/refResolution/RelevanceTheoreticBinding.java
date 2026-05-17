/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Variable;

/**
 * A candidate binding between a variable and a candidate
 *
 * @param variable  the name of the variable
 * @param candidate the candidate to be bound to that variable
 */
public class RelevanceTheoreticBinding {
  private final Variable variable;
  private final RelevanceTheoreticCandidate candidate;

  public RelevanceTheoreticBinding(Variable variable, RelevanceTheoreticCandidate candidate) {
    this.variable = variable;
    this.candidate = candidate;
  }

  public Variable variable() {
    return variable;
  }

  public RelevanceTheoreticCandidate candidate() {
    return candidate;
  }

  @Override
  public String toString() {
    return "RelevanceTheoreticBinding(" + variable + ", " + candidate + ")";
  }
}
