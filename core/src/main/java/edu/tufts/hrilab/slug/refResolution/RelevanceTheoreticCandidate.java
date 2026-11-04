/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Symbol;

/**
 * A resolution candidate
 */
public class RelevanceTheoreticCandidate {
  /**
   * a reference
   */
  private final Symbol ref;
  /**
   * the relevance of that candidate
   */
  private final double relevance;

  public RelevanceTheoreticCandidate(Symbol ref, double relevance) {
    this.ref = ref;
    this.relevance = relevance;
  }

  public Symbol ref() {
    return ref;
  }

  public double relevance() {
    return relevance;
  }

  @Override
  public String toString() {
    return "RelevanceTheoreticCandidate(" + ref + ", " + relevance + ")";
  }
}
