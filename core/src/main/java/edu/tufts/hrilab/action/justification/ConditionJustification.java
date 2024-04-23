/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.justification;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.util.Iterator;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ConditionJustification extends Justification {
  /**
   * The condition that either holds (value == true) or does not hold (value == false).
   * This is not the reason for failure or success.
   */
  private Predicate condition = null;
  /**
   * Binding options for any free-variables in the condition predicate.
   */
  private List<Map<Variable, Symbol>> bindings = new ArrayList<>();

  /**
   * Constructor indicating true/false without indicating a particular condition.
   * @param value
   */
  public ConditionJustification(boolean value) {
    super(value);
  }

  /**
   * Constructor indicating the logical truth value of the condition predicate, as well as the overall
   * value of the justification.
   * @param value
   * @param condition
   */
  public ConditionJustification(boolean value, Predicate condition) {
    super(value);
    this.condition = new Predicate(condition);
  }

  /**
   * Constructor indicating the logical truth value of the condition predicate, as well as the overall
   * value of the justification. The bindings contain binding options for any free-variables in the
   * condition predicate.
   * @param value
   * @param condition
   * @param bindings
   */
  public ConditionJustification(boolean value, Predicate condition, List<Map<Variable, Symbol>> bindings) {
    super(value);
    this.condition = new Predicate(condition);
    if (bindings != null) {
      this.bindings.addAll(bindings);
    }
  }

  @Override
  public List<Predicate> getPredicates() {
    List<Predicate> conditions = new ArrayList<>();
    if (condition != null) {
      conditions.add(condition);
    }
    return conditions;
  }

  @Override
  public List<Map<Variable, Symbol>> getBindings() {
    return bindings;
  }

  @Override
  public List<Predicate> getFailedConditions() {
    List<Predicate> failureConditions = new ArrayList<>();
    if (!getValue() && condition != null) {
      if (bindings.isEmpty()) {
        failureConditions.add(condition);
      } else {
        bindings.forEach(binding -> failureConditions.add(condition.copyWithNewBindings(binding)));
      }
    }
    return failureConditions;
  }

  @Override
  public List<Predicate> getFailureReason() {
    List<Predicate> failurePredicates = new ArrayList<>();
    getFailedConditions().forEach(cond -> failurePredicates.add(Factory.createNegatedPredicate(cond)));
    return failurePredicates;
  }

  @Override
  public List<Map<Variable,Symbol>> getFailureBindings() {
    if (!getValue()) {
      return bindings;
    }
    return new ArrayList<>();
  }

  @Override 
  public Set<Symbol> getBindings(Variable argument) {
    for (Symbol arg : condition.getArgs()) {
      if (arg instanceof Term) {
        for (Symbol tmpArg : ((Term) arg).getArgs()) {
          if (tmpArg.equals(argument)) {
            return getAvailableBindings(argument);
          }
        }
      } else {
        if (arg.equals(argument)) {
          return getAvailableBindings(argument);
        }
      }
    }
    return null;
  }

  private Set<Symbol> getAvailableBindings(Variable argument) {
    if (!getValue() || bindings == null) {
      return new HashSet<>();
    }
    Set<Symbol> availableBindings = new HashSet<>();
    for (Map<Variable, Symbol> binding : bindings) {
      availableBindings.add(binding.get(argument));
    }
    return availableBindings;
  }

  @Override
  public String toString() {
    if (condition != null) {
      return Factory.createPredicate("justifications", condition).toString();
    } else {
      return Factory.createPredicate("justifications()").toString();
    }
  }

  //todo: pete: this is implemented primarily so that justifications used as return types can be handled in the testing framework
  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    if (!super.equals(o)) return false;
    ConditionJustification justification = (ConditionJustification) o;
    if (condition != justification.condition) return false;
    if (!bindings.equals(justification.bindings)) return false;
    return true;
  }
}
