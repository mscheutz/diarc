/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Variable;

import java.util.LinkedList;
import java.util.List;

/**
 * A list of candidates to associate with a given variable
 */
public class SingleVarCandidateList {
  /**
   * The name of the variable in question
   */
  final Variable variable;
  /**
   * The list of mnemonic actions to take in searching for the referent of that variable
   */
  private final LinkedList<String> tiers;
  /**
   * The current list of candidates under consideration to be bound to that variable,
   * paired with their associated probability values
   */
  private final List<RelevanceTheoreticCandidateWithProabability> candidates;

  public SingleVarCandidateList(Variable variable, LinkedList<String> tiers, List<RelevanceTheoreticCandidateWithProabability> candidates) {
    this.variable = variable;
    this.tiers = tiers;
    this.candidates = candidates;
  }

  public Variable variable() {
    return variable;
  }

  public LinkedList<String> getTiers() {
    return tiers;
  }

  public List<RelevanceTheoreticCandidateWithProabability> getCandidates() {
    return candidates;
  }

  @Override
  public String toString() {
    return "SingleVarCandidateList(" + variable + ", " + tiers + ", " + candidates + ")";
  }
}
