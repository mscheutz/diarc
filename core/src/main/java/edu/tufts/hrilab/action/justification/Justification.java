/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author Tyler M. Frasca 
 */
package edu.tufts.hrilab.action.justification;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.io.Serializable;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Justification why a context has specific status
 * only stored for single status
 * getFailedPredicates, get predicates contributing to failure of context
 * getBindings, get bindings for predicates 
 */
public abstract class Justification implements Serializable {
  private boolean value;

  private Predicate step;

  public Justification() {
    value = false;
  }

  public Justification(boolean val) {
    value = val;
  }
  
  public final boolean getValue() {
    return value;
  }

  // TODO: make this protected
  public final void setValue(boolean value) {
    this.value = value;
  }

  public Predicate getStep() {
    return step;
  }

  public void setStep(Predicate step) {
    this.step = step;
  }

  /**
   *  Get the entire list of conditions within the justification
   *  @return list of conditions
   */
  public abstract List<Predicate> getPredicates();

  /**
   *  Get all the possible bindings for each condition within the justification
   *  @return list of the bindings for each condition
   */
  public abstract List<Map<Variable, Symbol>> getBindings();

  /**
   *  Get the conditions which do not hold. This will attempt to bind any free-variables in
   *  the failed conditions in the returned values. If multiple binding options exist, a single
   *  condition can produce multiple bound conditions.
   *
   *  @return list of failed predicates
   */
  public abstract List<Predicate> getFailedConditions();

  /**
   *  Get the conditions which hold in the environment, but cause the failure.
   *  This will attempt to bind any free-variables in the failed conditions in the returned values.
   *  If multiple binding options exist, a single condition can produce multiple bound conditions.
   *  @return list of predicates which hold in the environment
   */
  public abstract List<Predicate> getFailureReason();

  /**
   *  Get the bindings for the failed conditions.
   *  @return list of the bindings for each failed condition
   */
  public abstract List<Map<Variable, Symbol>> getFailureBindings();

  /**
   *  Get the bindings for a specific argument
   *  @param arg the arg to find bindings for
   *  @return set of bindings for the argument
   */
  public abstract Set<Symbol> getBindings(Variable arg);

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    Justification justification = (Justification) o;
    if (value != justification.value) return false;
    if ((step != null && justification.step == null) || (justification.step != null && step == null));
    if (step != null && justification.step != null && !step.equals(justification.step)) return false;
    return true;
  }
}
