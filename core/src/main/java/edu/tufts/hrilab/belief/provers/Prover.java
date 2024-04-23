/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Attempt at a unified prover interface for belief.
 *
 * @author luca
 * @author arindam
 */
package edu.tufts.hrilab.belief.provers;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.tuple.Pair;

import java.io.IOException;
import java.util.List;
import java.util.Map;

public interface Prover {

  /**
   * Available provers.
   */
  enum Engine {
    PROLOG,
    DCEC,
    SHADOW,
    CLINGO,
    DS
  }

  /**
   * checks validity of a belief.
   *
   * @param query The query / belief
   * @return whether the belief holds
   */
  Boolean querySupport(Term query);

  /**
   * checks validity of a belief, with an explanation from spy.
   *
   * @param query The query / belief
   * @return whether the belief holds

   */
  Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query);


  /**
   * Queries the Belief State about the term in the argument, and a set of
   * bindings corresponding to all of the found answers.
   *
   * @param query : a query in Term format
   * @return an ArrayList of bindings: here represented as a HashMap from
   * Variables to Symbols
   */
  List<Map<Variable, Symbol>> queryBelief(Term query);

  /**
   * Adds a single belief to the Belief State of the agent. Note:
   * new beliefs are added such that they take priority in unification to old
   * beliefs. queryBelief() will still return all asserted and inferred
   * bindings.
   *
   * @param belief
   */
  void assertBelief(Term belief);

  /**
   * Removes a single belief from the Belief State of the agent.
   *
   * @param belief
   */
  void retractBelief(Term belief);

  /**
   * @param head
   * @param body
   */
  void assertRule(Term head, List<Term> body);

  /**
   * remove the specified rule
   */
  void retractRule(Term head, List<Term> body);

  /**
   * initialize the knowledge base from a file
   */
  void initializeFromFiles(List<String> filenames) throws IOException;

  /**
   * Get the theory representation as a string in target language format.
   * Returned string depends on prover implementation.
   *
   * @return internal theory representation
   */
  String getTheory();

  /**
   * Get the theory representation in its source format
   *
   * @return internal theory representation
   */
  Object getInternalTheory();

  /**
   * Set the theory (aka beliefs aka assumptions).
   *
   * @return true if theory was set successfully
   */
  boolean setTheory(String theory);

  /**
   * Assert belief to prover.
   * @param belief
   */
  void assertBelief(String belief);

  /**
   * Retract belief from prover.
   * @param belief
   */
  void retractBelief(String belief);

  /**
   * Add rule to prover.
   * @param rule
   */
  void assertRule(String rule);

  /**
   * Retract rule from prover.
   * @param rule
   */
  void retractRule(String rule);

  /**
   * Convert DIARC FOL Term to appropriate prover syntax.
   * TODO: consolidate all code related to converting FOL to/from Prover
   *       and replace this method with two new methods (1) convertToProverForm
   *       and (2) convertToDiarcForm
   * @param term
   * @return
   */
  String sanitize(Term term);
}
