/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.fol;

import java.io.Serializable;
import java.util.Objects;

/**
 * Representation of variable logical form.
 */
public class Variable extends Symbol implements Cloneable, Serializable {

  private Symbol value = null;
  static final long serialVersionUID = 4085117658898612398L;
  @Deprecated
  public void setType(String t) {
    type = t;
  }

  public Symbol getValue() {
    return value;
  }

  public Variable(Variable v) {
    super(v.getName(), v.getType());
    this.value = v.getValue();
  }

  public Variable(String name) {
    super(name);
  }

  public Variable(String name, String type) {
    super(name, type);
  }

  //Brad: I think it has been decided this isn't how we want to do this?
  @Deprecated
  public Variable(String name, String type, Symbol value) {
    super(name, type);
    this.value = value;
  }

  @Override
  public boolean isVariable() {
    return true;
  }

  /**
   * Compare without checking semantic type information.
   * @param o
   * @return
   */
  public boolean equalsIgnoreType(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    if (!super.equalsIgnoreType(o)) return false;
    Variable variable = (Variable) o;
    return value.equalsIgnoreType(variable.value);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    if (!super.equals(o)) return false;
    Variable variable = (Variable) o;
    return Objects.equals(value, variable.value);
  }

  @Override
  public int hashCode() {
    return Objects.hash(super.hashCode(), value);
  }

  @Override
  public Variable clone() {
    Variable clone = (Variable) super.clone();
    if (value != null) {
      clone.value = value.clone();
    }
    return clone;
  }
}
