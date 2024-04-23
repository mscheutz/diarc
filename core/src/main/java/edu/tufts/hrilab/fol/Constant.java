/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.fol;

import java.io.Serializable;
import java.util.Objects;

/**
 * Representation of constant logical form.
 */
public class Constant extends Symbol implements Cloneable, Serializable {

  static final long serialVersionUID =-571526824461435393L;

  private Object value = null;

  public Object getValue() {
    return value;
  }

  @Override
  public Constant clone() {
    return (Constant) super.clone();
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
    Constant constant = (Constant) o;
    return constant.equalsIgnoreType(constant.value);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    if (!super.equals(o)) return false;
    Constant constant = (Constant) o;
    return Objects.equals(value, constant.value);
  }

  @Override
  public int hashCode() {
    return Objects.hash(super.hashCode(), value);
  }

  public Constant(Constant c) {
    super(c.getName(), c.getType());
    value = c.getValue();
  }

  public Constant(String input) {
    super(input);
  }

  public Constant(String name, String type) {
    super(name, type);
  }

  public Constant(String name, String type, Object value) {
    super(name, type);
    this.value = value;
  }

  @Override
  public boolean isConstant() {
    return true;
  }

}
