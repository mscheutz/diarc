/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.fol;

import java.io.Serializable;
import java.util.Collection;
import java.util.List;
import java.util.Map;

/**
 * Representation of predicate logical form.
 */
public class Predicate extends Term implements Cloneable, Serializable {

  static final long serialVersionUID =-5670693651597478767L;//3530131803247277054L;

  public Predicate(Term t) {
    super(t);
  }

  public Predicate(String n, List<? extends Symbol> a) {
    super(n, a);
  }

  public Predicate(Symbol n, List<? extends Symbol> a) {
    super(n, a);
  }

  public Predicate(String n, Symbol... a) {
    super(n, a);
  }

  public Predicate(String n, String... a) {
    super(n, a);
  }

  @Override
  public Predicate clone() {
    return (Predicate) super.clone();
  }

  @Override
  public boolean isPredicate() {
    return true;
  }

  @Override
  public Predicate toNegatedForm() {
    if (name.equals("not")) {
      // already negated
      return this.clone();
    } else {
      return new Predicate("not", this.clone());
    }
  }

  /**
   * Similar to applyBindingMap, except this Term is not bound. Instead,
   * a cloned version is bound and returned.
   * @param bindings
   * @return
   */
  @Override
  public Predicate copyWithNewBindings(Map<Variable, ? extends Symbol> bindings) {
    Predicate p = this.clone();
    p.applyBindingMap(bindings);
    return p;
  }

  /**
   * Override Term's implementation to return Predicate.
   * @param typedVariables
   * @return
   */
  @Override
  public Predicate copyWithNewVariableTypes(Collection<Variable> typedVariables) {
    Predicate p = this.clone();
    p.applyVariableTypes(typedVariables);
    return p;
  }
}
