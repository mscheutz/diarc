/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.lang.Double;
import java.util.List;
import java.util.Map;

import ai.thinkingrobots.trade.TRADEService;

/**
 * ConsultantInterface is the base set of methods for reference resolution (i.e., POWER) in DIARC.
 */
public interface ConsultantInterface {
  /**
   * Returns name of things handles by this consultant (e.g., "objects", "locations", etc).
   *
   * @return knowledge base name
   */
  @TRADEService
  String getKBName();

  /**
   * Returns a list of Properties handled by this component.
   *
   * @return list of Properties.
   */
  @TRADEService
  List<Term> getPropertiesHandled();

  /**
   * Returns a list of currently known refIds (it takes a seq of Properties, to optionally
   * do some pre-processing to return a list of relevant candidates).
   *
   * @param query properties to optionally do some pre-processing
   * @return list of currently known ids
   */
  //TODO:brad: this should probably be two separate methods,
  // one that takes no args and returns everything,
  // and one that takes args, and returns a filtered list
  // or maybe never do the filtering as part of the consultant
  @TRADEService
  List<Symbol> getInitialDomain(List<Term> query);

  /**
   * Returns the probability that a property (aka constraint) holds for
   * an entity (specified by a map of free-variable to refId bindings).
   * <p>
   * TODO: rename method to something more descriptive. how about getProbability or checkProperty?
   *
   * @param constraint Property to check
   * @param bindings   Map of free variables to refIds
   * @return probability value [0.0. 1.0]
   */
  @TRADEService
  Double process(Term constraint, Map<Variable, Symbol> bindings);

  /**
   * new representations are created for each free-variable and returned
   * in the binding list.
   *
   * @param vars list of free-variables
   * @return list of bindings
   */
  @TRADEService
  Map<Variable, Symbol> createReferences(List<Variable> vars);

  /**
   * an attempt is made to bring the knowledge base in line with the
   * provided bindings, list of properties, and probability value.
   *
   * @param bindings   list of bindings in tuple form (free-variable and refId pair)
   * @param prob       probability value
   * @param properties list of Properties
   * @return whether the updated was successful?
   */
  @TRADEService
  boolean assertProperties(Map<Variable, Symbol> bindings, Double prob, List<Term> properties);

  /**
   * Get a list of currently highly salient entities that should be considered "activated."
   *
   * @return list of refIds and probability pairs
   */
  @TRADEService
  Map<Symbol, Double> getActivatedEntities();

  /**
   * Get the list of properties that have been asserted for a given refId.
   *
   * @param refId reference resolution id
   * @return list of Properties
   */
  @TRADEService
  List<Term> getAssertedProperties(Symbol refId);

  //brad:called by the RR component assert properties which is an Action
  //TODO:Brad: I'm not sure if this should exist. I think that the RR component method with this
  // signature could just used the existing assert properties under the covers. This is over ridden
  // in a few places, though and I haven't ahd time to see if there is anything different going on
  // there compared with the other assert properties.

  /**
   * Add properties to already existing object ref. Each property should contain a connection
   * to the existing reference either by refId or refId's variable.
   *
   * @param refId      reference
   * @param properties reference properties
   * @return if property was successfully added to reference refId
   */
  @TRADEService
  boolean assertProperties(Symbol refId, List<Term> properties);

  /**
   * Convert refId to target class. This is a way to map references onto Java-types. If it's
   * not possible for a refId to be converted to the specified type, null is returned.
   *
   * @param refId - reference resolution ID (e.g.object_3)
   * @param type  - target Java class type
   * @param <U>   target Java class template type (any)
   * @return instance of specified target Java class
   */
  @TRADEService
  <U> U convertToType(Symbol refId, Class<U> type);

  /**
   * Convert refId to target class using the specified constraints for to filter the results.
   * This is a way to map references onto Java-types. If it's
   * not possible for a refId to be converted to the specified type, null is returned.
   *
   * @param refId       - reference resolution ID (e.g.object_3)
   * @param type        - target Java class type
   * @param constraints first-order-logic predicate constraints
   * @return instance of specified target Java class
   * @param <U>         target Java class template type (any)
   */
  @TRADEService
  <U> U convertToType(Symbol refId, Class<U> type, List<? extends Term> constraints);

  @TRADEService
  void registerForNewPropertyNotification(TRADEServiceInfo callback);

  @TRADEService
  void unregisterForNewPropertyNotification(TRADEServiceInfo callback);

  void notifyNewPropertySubscribers();
}


