/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

/**
 * Straightforward: A resolution candidate paired with a probability value.
 */
public class RelevanceTheoreticCandidateWithProabability {
  private final RelevanceTheoreticCandidate candidate;
  private final double probability;

  public RelevanceTheoreticCandidateWithProabability(RelevanceTheoreticCandidate candidate, double probability) {
    this.candidate = candidate;
    this.probability = probability;
  }

  public RelevanceTheoreticCandidate candidate() {
    return candidate;
  }

  public double probability() {
    return probability;
  }

  @Override
  public String toString() {
    return "RelevanceTheoreticCandidateWithProabability(" + candidate + ", " + probability + ")";
  }
}
