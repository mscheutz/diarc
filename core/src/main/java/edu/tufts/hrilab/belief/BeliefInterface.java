/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief;

import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.action.annotations.Action;
import org.apache.commons.lang3.tuple.Pair;
import java.util.Set;
import java.util.Map;
import java.util.List;

import ai.thinkingrobots.trade.*;

//TODO:brad: I don't think this interface needs to exist, given that the prover interface exists. All of these annotations and docs should jet get moved to the Impl

/** Note: The maps of bindings from Variables to Lists of Symbols are here
 *  represented by {@code HashMap<Variable, ArrayList<Symbol>>}.
 */
public interface BeliefInterface {
  /**
   * Register to be notified by Belief whenever the query term is true. The callback
   * method must have the following method signature to receive callbacks:
   * {@code callbackMethod(Term queryTerm, List<Map<Variable, List<Symbol>>> bindings)}
   * If the query contains no variables,
   * a list containing one empty map will be returned for true queries.
   * @param queryTerm
   * @param callback
   */
  @TRADEService
  void registerForNotification(Term queryTerm, TRADEServiceInfo callback);

  /**
   * Unregister to be notified by Belief for a particular query term.
   * @param queryTerm
   * @param callback
   */
  @TRADEService
  void unregisterForNotification(Term queryTerm, TRADEServiceInfo callback);

  /**
   * Adds the beliefs in the argument to the Belief State of the agent. Note:
   * new beliefs are added such that they take priority in unification to old
   * beliefs. queryBelief() will still return all asserted and inferred
   * bindings.
   *
   *  @param beliefs : set of beliefs to add to the Belief State.
   */
  @TRADEService
  @Action
  void assertBeliefs(Set<Term> beliefs, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  void assertBeliefs(Set<Term> beliefs);

  /**
   * Adds a single belief to the Belief State of the agent. Note:
   * new beliefs are added such that they take priority in unification to old
   * beliefs. queryBelief() will still return all asserted and inferred
   * bindings.
   * @param belief
   */
  @TRADEService
  @Action
  void assertBelief(Term belief, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  void assertBelief(Term belief);

  /**
   * Removes the beliefs in the argument from the Belief State of the agent.
   *
   *  @param beliefs : set of beliefs to be removed
   */
  @TRADEService
  @Action
  void retractBeliefs(Set<Term> beliefs, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  void retractBeliefs(Set<Term> beliefs);

  /**
   * Removes a single belief from the Belief State of the agent.
   * 
   * @param belief
   */
  @TRADEService
  @Action
  void retractBelief(Term belief, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  void retractBelief(Term belief);

  /**
   * Asserts a rule with one premise and a set of conclusions into the Belief State of the agent.
   * @param head
   * @param body
   */
  @TRADEService
  @Action
  void assertRule(Term head, List<Term> body, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  void assertRule(Term head, List<Term> body);

  /**
   * Retracts a rule containing any number of premises and any number of conclusions.
   * @param head
   * @param body
   */
  @TRADEService
  @Action
  void retractRule(Term head, List<Term> body, MemoryLevel memoryLevel);

  /**
   * Retracts a rule by its head.
   * @param head
   */
  @TRADEService
  @Action
  void retractRule(Term head);

  @TRADEService
  @Action
  void retractRule(Term head, List<Term> body);

  /**
   * Returns list of current facts in predicate form.
   * This returns facts that are explicitly true, and facts
   * that are true as a result of inference, using facts
   * and rules from specified MemoryLevel.
   * @return Facts
   */
  @TRADEService
  List<Term> getFacts(MemoryLevel memoryLevel);

  /**
   * Returns list of current facts in predicate form.
   * This returns facts that are explicitly true, and facts
   * that are true as a result of inference.
   * @return Facts
   */
  @TRADEService
  List<Term> getFacts();

  /**
   * Returns list of current rules in predicate form, head first (conclusion) and then body (premises).
   * @return List of pairs of rulehead and rulebody.
   */
  @TRADEService
  List<Pair<Term,List<Term>>> getRules(MemoryLevel memoryLevel);

  @TRADEService
  List<Pair<Term,List<Term>>> getRules();

  /**
   * Returns list of past facts in predicate form, paired with their assertion and retraction times.
   * @return List of pairs of facts and start/end times.
   */
  @TRADEService
  List<Pair<Term,Pair<Symbol,Symbol>>> getHistory(MemoryLevel memoryLevel);

  @TRADEService
  List<Pair<Term,Pair<Symbol,Symbol>>> getHistory();

  /**
   * Queries the Belief State about the term in the argument, and a set of
   * bindings corresponding to all of the found answers.
   *
   *  @param query : a query in Term format
   *  @return an ArrayList of bindings: here represented as a HashMap from
   *          Variables to Symbols
   */
  @TRADEService
  List<Map<Variable, Symbol>> queryBelief(Term query, MemoryLevel memoryLevel);



  @TRADEService
  @Action
  List<Map<Variable, Symbol>> queryBelief(Term query);

  /**
   * Queries the Belief State with N independent queries, and returns N binding lists.
   *
   *  @param queries : a list of N queries in Term format
   *  @return an List of N lists of bindings: here represented as a HashMap from
   *          Variables to Symbols
   */
  @TRADEService
  @Action
  List<List<Map<Variable, Symbol>>> queryBelief(List<Term> queries, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  List<List<Map<Variable, Symbol>>> queryBelief(List<Term> queries);

  /**
   * Make query and return if query has support (is true).
   * @param query
   */
  @TRADEService
  @Action
  Boolean querySupport(Term query, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  Boolean querySupport(Term query);

  /**
   * Queries true or false
   * @param query
   * @return True or false, paired with a map of facts and booleans which
   * lead to the original query being true or false.
   */
  @TRADEService
  @Action
  Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query, MemoryLevel memoryLevel);

  @TRADEService
  @Action
  Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query) ;

  /**
   * Make query using last n facts most recently asserted to the database
   * (out of all facts, not necessarily currently true ones).
   * @return Query results
   */
  @TRADEService
  List<Map<Variable, Symbol>> queryRecentFacts(Term query, int n);

  /**
   * Make query using all facts that were true at time t.
   * @return Query results
   */
  @TRADEService
  List<Map<Variable, Symbol>> queryAtTime(Term query, Long time);

  /**
   * Make query using facts which were true from start to end of timeframe.
   * @return Query results
   */
  @TRADEService
  List<Map<Variable, Symbol>> queryFactsInTimeframe(Term query, Long start, Long end);

  /**
   * Query for how long ago a belief was asserted and retracted.
   * @return Query results
   */
  @TRADEService
  @Action
  Map<Term,List<Pair<Long,Long>>> queryRecency(Term query);

  @TRADEService
  void clear();

  /**
   * Clear beliefs at specified memory level only.
   * @param memoryLevel
   */
  @TRADEService
  void clear(MemoryLevel memoryLevel);

//  /**
//   * Create a new table with the requested tblname.
//   * @return ResultSet containing facts
//   * @throws SQLException
//   */
//  @TRADEService
//  void createTopic(String topic);
//
//  /**
//   * Swaps out current prover space for the requested SQL table.
//   * @throws SQLException
//   */
//  @TRADEService
//  void changeTopic(String topic);
//
//  /**
//   * Adds content of requested SQL table to current prover space.
//   * @throws SQLException
//   */
//  @TRADEService
//  void appendTopic(String topic);
//
//  /**
//   * Loads prover with beliefs from table which contain a specific word
//   * Loads new table with name tblnameWITHfilter
//   * @throws SQLException
//   */
//  @TRADEService
//  void appendTopic(String topic, String filter);

  /**
   * Copy the prover and sql state to a new BeliefComponent instance and return it. This should not
   * automatically register the new component with TRADE, and is up to the caller to register
   * if desired.
   * @return
   */
  BeliefInterface createClone();
}
