/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution

import edu.tufts.hrilab.fol.{Variable,Symbol}

import scala.collection.mutable.ListBuffer

/**
* A resolution candidate
 *
* @param ref a reference
* @param relevance the relevance of that candidate
*/
case class RelevanceTheoreticCandidate(ref:Symbol, relevance:Double)

/**
  * Straightforward: A resolution candidate paired with a probability value.
  *
  * @param candidate
  * @param probability
  */
case class RelevanceTheoreticCandidateWithProabability(candidate:RelevanceTheoreticCandidate, probability:Double)

/**
  * A candidate binding between a variable and a candidate
  *
  * @param variable the name of the variable
  * @param candidate the candidate to be bound to that variable
  */
case class RelevanceTheoreticBinding(variable: Variable, candidate: RelevanceTheoreticCandidate)

/**
  * A list of candidates to associate with a given variable
  *
  * @param variable The name of the variable in question
  * @param tiers The list of actions to mnemonic actions to take in searching for the referent of that variable
  * @param candidates The current list of candidates under consideration to be bound to that variable,
  * paired with their associated probability values
  */
case class SingleVarCandidateList(variable: Variable, tiers: ListBuffer[String], candidates: ListBuffer[RelevanceTheoreticCandidateWithProabability])

/**
  * The result of combining together multiple single-variable candidate lists: a set of candidate variable-reference bindings,
  * and an aggregate probability for that joint mapping.
  *
  * @param bindings
  * @param probability
  */
case class CrossMappingCandidateList(bindings:List[RelevanceTheoreticBinding], probability:Double)