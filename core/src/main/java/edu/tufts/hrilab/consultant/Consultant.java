/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.consultant;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.consultant.util.Utilities;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.interfaces.ConsultantInterface;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import org.apache.commons.lang3.StringUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.*;
import java.util.stream.Collectors;

public abstract class Consultant<T extends Reference> implements ConsultantInterface {
  protected Logger log = LoggerFactory.getLogger(this.getClass());

  /**
   * Reference counter used to assign unique refIds.
   */
  protected int refNumber = 0;
  /**
   * All properties that can be handled by this consultant, excluding the defaultProperties.
   */
  protected List<Term> propertiesHandled;
  /**
   * These properties implicitly apply to every reference in this consultant, even though they will
   * not appear as explicit properties in the reference.
   */
  protected List<Term> defaultProperties;
  /**
   * All References hashed refId.
   */
  protected Map<Symbol, T> references;
  /**
   * Knowledge base name.
   */
  protected String kbName;
  /**
   * Constructor for templated Reference class.
   */
  protected Constructor<T> refConstructor;

  /**
   * Subscribers that must be notified when a new property is added to the consultant.
   */
  private HashSet<TRADEServiceInfo> propertyNotificationSubscribers;

  // TODO: consider replacing Class<T> with Factory<T> interface.

  /**
   * @param refClass - Reference class. Used to construct new instances of Reference.
   * @param kbName   knowledge base name
   */
  public Consultant(Class<T> refClass, String kbName) {
    try {
      Class[] cArgs = new Class[]{Symbol.class, Variable.class};
      refConstructor = refClass.getDeclaredConstructor(cArgs);
    } catch (NoSuchMethodException e) {
      log.error("Reference class doesn't contain expected constructor signature: " + refClass);
    }

    this.kbName = kbName;
    this.references = new HashMap<>();
    this.propertiesHandled = new ArrayList<>(Arrays.asList(Factory.createPredicate("this(X:" + kbName + ")")));
    this.defaultProperties = Arrays.asList(
            Factory.createPredicate("it(X:" + kbName + ")"),
            Factory.createPredicate("that(X:" + kbName + ")"),
            Factory.createPredicate("thing(X:" + kbName + ")"),
            Factory.createPredicate("those(X:" + kbName + ")"),
            Factory.createPredicate("they(X:" + kbName + ")"),
            Factory.createPredicate("these(X:" + kbName + ")"));
    this.propertyNotificationSubscribers = new HashSet<>();
  }

  public Consultant(Class<T> refClass, String kbName, List<String> properties) {
    this(refClass, kbName);
    addPropertiesHandled(properties.stream().map(p -> Factory.createPredicate(p, "X:" + kbName)).collect(Collectors.toList()));
  }

  @Override
  public String getKBName() {
    return kbName;
  }

  @Override
  public List<Term> getPropertiesHandled() {
    List<Term> properties = new ArrayList<>();
    properties.addAll(propertiesHandled);
    properties.addAll(defaultProperties);
    return properties;
  }

  /**
   * Updates consultant and parser if any of the members of @properties are not already handled by the Consultant
   *
   * @param properties list of candidate properties
   * @return true if propertiesHandled is modified
   */
  public boolean addPropertiesHandled(List<Term> properties) {
    boolean propertiesAdded = false;
    for (Term property : properties) {
      if (propertiesHandled.stream().noneMatch(p -> edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(p, property))) {
        propertiesHandled.add(property);
        propertiesAdded = true;
        String propName = property.getName();
        String strippedName = stripQuotesFromMorpheme(propName);

        // TODO: this imposes an ordering constraint on diarc components (i.e., RR needs to be up first)
        List<TRADEServiceInfo> injectTSIs = new ArrayList<>(TRADE.getAvailableServices(new TRADEServiceConstraints().name("injectDictionaryEntry")));
        if (!injectTSIs.isEmpty()) {
          try {
            TRADEServiceInfo injectTSI = injectTSIs.get(0);
            injectTSI.call(void.class, strippedName, "RN", propName + ":" + kbName, "VAR");
            injectTSI.call(void.class, strippedName, "REF", propName + ":" + kbName, "DEFINITE");
            injectTSI.call(void.class, strippedName, "DESC", propName + ":property", "");
            injectTSI.call(void.class, strippedName, "DN", propName + ":property", "");
          } catch (TRADEException e) {
            log.error("[addPropertiesHandled] unable to add dictionary entry for " + propName, e);
          }
        }

      }
    }
    if (propertiesAdded) {
      notifyNewPropertySubscribers();
    }
    return propertiesAdded;
  }

  /**
   * Add new reference from some source (file or learning) to the consultant
   *
   * @param ref new reference to add
   */
  public void addReference(T ref) {
    references.put(ref.refId, ref);
    addPropertiesHandled(ref.properties);
  }

  public boolean retractProperties(Symbol refId, List<Term> properties) {
    Reference ref = references.get(refId);
    if (ref == null) {
      log.error("[retractProperties] couldn't find ref: " + refId);
      return false;
    }

    log.debug("[retractProperties] attempting to retract properties: " + properties);

    // replace all occurrences of refId with ref's variable
    // e.g., on(grasp_point, objects_0) --> on(grasp_point, X)
    Variable var = ref.variable;
    List<Term> convertedProperties = new ArrayList<>();
    for (Term property : properties) {
      Predicate convertedProperty = PredicateHelper.replace(property, refId, var);
      convertedProperties.add(convertedProperty);
    }

    log.debug("[retractProperties] retracting properties: " + convertedProperties);
    int numPropertiesRemoved = 0;
    for (Term property : convertedProperties) {
      for (Term refProperty : ref.properties) {
        if (edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(refProperty, property)) {
          ref.properties.remove(refProperty);
          ++numPropertiesRemoved;
          break; // from inner loop
        }
      }
    }

    if (numPropertiesRemoved == properties.size()) {
      return true;
    } else {
      log.warn("[retractProperties] did not successfully remove all properties.");
      return false;
    }
  }

  //TODO: I don't think we need to retain the functionality where this method takes in a constraints and returns a contained initial domain. I think we can remove the arg from this method entirely.
  @Override
  public List<Symbol> getInitialDomain(List<Term> query) {
    if (query.isEmpty()) {
      return new ArrayList<>(references.keySet());
    } else {
      throw new UnsupportedOperationException("[getInitialDomain] can't take in a non-empty query right now");
    }
  }

  @Override
  public Double process(Term constraint, Map<Variable, Symbol> bindings) {
    log.debug("Being asked how probable it is that " + constraint + " holds under bindings " + bindings);

    //TODO: To get the reference, we should look at the variable(s) in the constraint and use them to get the associated
    // bindings and perhaps throw an error if the variables in the constraint dont match the keys in bindings.
    //get reference
    Symbol refId = (Symbol) bindings.values().toArray()[0];
    T reference = references.get(refId);

    if (reference == null) {
      log.warn("[process] " + getKBName() + " cannot find refId: " + refId);
      return 0.0;
    }

    log.debug("Reference properties: " + reference.properties);
    for (Term property : reference.properties) {
      if (edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(property, constraint)) {
        log.debug("Reference contains property: " + constraint);
        return 1.0;
      }
    }

    // check against default vision consultant properties (e.g., it, that, ...)
    for (Term property : defaultProperties) {
      if (edu.tufts.hrilab.fol.util.Utilities.predicatesMatch(property, constraint)) {
        return 1.0;
      }
    }

    log.debug("Reference does NOT contain property: " + constraint);
    return 0.0;
  }

  @Override
  public Map<Symbol, Double> getActivatedEntities() {
    log.debug("[getActivatedEntities]");
    Map<Symbol, Double> vals = new HashMap<>();
    for (Symbol ref : references.keySet()) {
      vals.put(ref, 10.0);
    }
    return vals;
  }

  @Override
  public List<Term> getAssertedProperties(Symbol refId) {
    log.debug("[getAssertedProperties] trying to get properties for: " + refId);
    T ref = references.get(refId);
    if (ref != null) {
      log.debug("properties: " + ref.properties);
      return ref.properties;
    }
    return new ArrayList<>();
  }

  /**
   * A convenience method for creating references from with a DIARC component housing a Consultant instance.
   *
   * @param var        Variable associated with new reference (only important if properties contain more than one Variable)
   * @param properties semantic properties of reference
   * @return reference of type T
   */
  public T createReference(Variable var, List<Term> properties) {
    T newRef = null;

    if (refConstructor == null) {
      log.error("[createReferences] Reference constructor is null. Cannot create new References.");
      return newRef;
    }

    Symbol refId = getNextReferenceId();
    try {
      newRef = refConstructor.newInstance(refId, var);
      newRef.properties.addAll(properties);
      references.put(refId, newRef);
      addPropertiesHandled(properties);
    } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
      log.error("[createReferences] Error constructing new Reference.", e);
    }

    return newRef;
  }

  @Override
  public Map<Variable, Symbol> createReferences(List<Variable> vars) {
    Map<Variable, Symbol> refs = new HashMap<>();
    if (refConstructor == null) {
      log.error("[createReferences] Reference constructor is null. Cannot create new References.");
      return refs;
    }

    Symbol refId;
    for (Variable var : vars) {
      refId = getNextReferenceId();
      try {
        T ref = refConstructor.newInstance(refId, var);
        references.put(refId, ref);
        refs.put(var, refId);
      } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
        log.error("[createReferences] Error constructing new Reference.", e);
      }
    }

    if (log.isDebugEnabled()) {
      log.debug("New refs = " + refs);
      log.debug("All refs = " + references);
    }
    return refs;
  }

  @Override
  public boolean assertProperties(Map<Variable, Symbol> bindings, Double prob, List<Term> properties) {
    for (Symbol refId : bindings.values()) {
      if (references.containsKey(refId)) {
        Reference ref = references.get(refId);
        ref.properties.addAll(properties);
        return true;
      } else {
        //TODO:brad: maybe this is fine and we don't need the warning? Still not sure if this is where this decision should be made, though.
        log.debug("[assertProperties] consultant: " + getKBName() + " doesn't contain ref: " + refId);
        return false;
      }
    }
    return true;
  }

  @Override
  public boolean assertProperties(Symbol refId, List<Term> properties) {
    log.debug("[assertProperties] refId: " + refId + " properties: " + properties);

    Reference ref = references.get(refId);
    if (ref == null) {
      log.error("[assertProperties] couldn't find existing ref: " + refId);
      return false;
    }
    log.debug("[assertProperties] refId: " + refId + " pre assert ref properties: " + ref.properties);
    ref.properties.addAll(properties);
    log.debug("[assertProperties] refId: " + refId + " post assert ref properties: " + ref.properties);
    return true;
  }

  /**
   * Helper method to generate next unique reference ID (e.g., object_3).
   *
   * @return unique reference ID
   */
  protected Symbol getNextReferenceId() {
    //todo: Now that this has type information, do we still care about kbname? -MF
    return Factory.createSymbol(kbName + "_" + refNumber++, kbName);
  }

  public String getReferenceSummaries() {
    StringBuilder sb = new StringBuilder();
    for (Symbol refId : references.keySet()) {
      sb.append(refId).append(" = ").append(references.get(refId).properties).append("\n");
    }
    return sb.toString();
  }

  @TRADEService
  //todo: this was previously `final`. removed to implement reference sharing on specific consultants. should that be generalized.
  public T getReference(Symbol refId) {
    return references.get(refId);
  }

  /**
   * Get all references that have all of the given properties
   *
   * @param properties property to search for
   * @return list of references
   */
  public List<Symbol> getReferencesWithAllProperties(List<Term> properties) {
    List<Symbol> refs = new ArrayList<>();
    for (T reference : references.values()) {
      if (new HashSet<>(reference.properties).containsAll(properties)) {
        refs.add(reference.refId);
      }
    }
    return refs;
  }

  /**
   * Get all references that have any of the given properties
   *
   * @param properties property to search for
   * @return list of references
   */
  public List<Symbol> getReferencesWithAnyProperties(List<Term> properties) {
    List<Symbol> refs = new ArrayList<>();
    for (T reference : references.values()) {
      if (!Collections.disjoint(reference.properties, properties)) {
        refs.add(reference.refId);
      }
    }
    return refs;
  }

  @Override
  final public <U> U convertToType(Symbol refId, Class<U> type) {
    return convertToType(refId, type, new ArrayList<>());
  }


  @Override
  final public <U> U convertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
    return localConvertToType(refId, type, constraints);
  }


  /**
   * The local implementation of convertToType.
   *
   * @param refId reference resolution ID (e.g.object_3)
   * @param type  target Java class type
   * @param <U>   target Java class template type (any)
   * @return instance of specified target Java class
   */
  abstract protected <U> U localConvertToType(Symbol refId, Class<U> type);

  /**
   * The local implementation of convertToType.
   *
   * @param refId       reference resolution ID (e.g.object_3)
   * @param type        target Java class type
   * @param constraints first-order-logic predicate constraints
   * @param <U>         target Java class template type (any)
   * @return instance of specified target Java class
   */
  abstract protected <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints);


  /**
   * Register to be notified when new properties are added to this consultant.
   *
   * @param callback TRADE service to be notified at.
   */
  @Override
  public void registerForNewPropertyNotification(TRADEServiceInfo callback) {
    propertyNotificationSubscribers.add(callback);
  }

  /**
   * Un-register from new property notifications
   *
   * @param callback TRADE service that was registered
   */
  @Override
  public void unregisterForNewPropertyNotification(TRADEServiceInfo callback) {
    propertyNotificationSubscribers.remove(callback);
  }

  /**
   * Notify all new property subscribers.
   */
  @Override
  public void notifyNewPropertySubscribers() {
    for (TRADEServiceInfo subscriber : propertyNotificationSubscribers) {
      try {
        subscriber.call(void.class);
      } catch (TRADEException e) {
        log.error("Could not make callback to: " + subscriber, e);
      }
    }
  }

  /**
   * Remove quotes from input String.
   * <p>
   * TODO: move this method to utility class
   *
   * @param input String with quotes
   * @return string without quotes
   */
  public String stripQuotesFromMorpheme(String input) {
    int escapeStart = -1;
    int escapeEnd = -1;
    for (int i = 0; i < input.length(); i++) {
      if (input.charAt(i) == '\"') {
        if (escapeStart == -1) {
          escapeStart = i;
        } else {
          escapeEnd = i;
        }
      }
    }
    if (escapeStart == 0 && escapeEnd == (input.length() - 1)) {
      return input.substring(1, input.length() - 1);
    } else {
      if (escapeStart != -1 || escapeEnd != -1) {
        log.warn("[stripQuotesFromMorpheme] escape characters appear in positions other than the beginning and end of the literal: " + input);
      }
      return input;
    }
  }
}