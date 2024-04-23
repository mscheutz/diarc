
/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.fol;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.Serializable;
import java.util.Objects;

/**
 * Representation of symbol logical form.
 */
public class Symbol implements Cloneable, Serializable {
  private static final Logger log = LoggerFactory.getLogger(Symbol.class);
  protected String name;
  protected String type;
  static final long serialVersionUID =-2044814809823357783L;

  /**
   * Get name as String.
   * @return
   */
  public String getName() {
    return name;
  }

  /**
   * Set name.
   * @param n
   */
  @Deprecated
  public void setName(String n) {
    name = n;
  }

  /**
   * Get type as String.
   * @return
   */
  public String getType() {
    return type;
  }

  /**
   * Returns if type has been set.
   * @return
   */
  public boolean hasType() {
    return type != null && !type.isEmpty();
  }

  /**
   * Compare without checking semantic type information.
   * @param o
   * @return
   */
  public boolean equalsIgnoreType(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    Symbol symbol = (Symbol) o;
    return name.equals(symbol.name);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    Symbol symbol = (Symbol) o;
    return name.equals(symbol.name) &&
            type.equals(symbol.type);
  }

  @Override
  public int hashCode() {
    return Objects.hash(name, type);
  }

  /**
   * Constructs a String representation that includes type information, if available.
   * If no type information is available it will be ignored.
   *
   * Strings will be of the form: "name:type(arg0:typ0,..., argN:typeN)".
   *
   * @return
   */
  @Override
  public String toString() {
    if (type.isEmpty()) {
      return name;
    } else {
      StringBuilder sb = new StringBuilder();
      sb.append(name).append(":").append(type);
      return sb.toString();
    }
  }

  /**
   * Constructs a String representation that does not include type information.
   *
   * @return String of the form "name(arg0,..., argN)"
   */
  public String toUntypedString() {
    return name;
  }

  /**
   * Returns a FOL instance that is not negated (i.e., drops outer "not" if it exists). This doesn't
   * apply to the Symbol class which can't be negated, but is here as a convenience so that Term/Predicate
   * instances passed as Symbols (e.g., via getArgs) don't need to be cast to Term/Predicate.
   *
   * @return Un-negated form of this FOL class.
   *
   */
  public Symbol toUnnegatedForm() {
    return this;
  }

  //TODO:brad:why do we have this and clone?
  public Symbol(Symbol s) {
    this.name = s.name;
    this.type = s.type;
  }

  //The string syntax that uses a colon should be passed to Factory.create Symbol
  //Other cases should provide a type

  public Symbol(String input) {
    String[] split = input.split(":");
    this.name = split[0];
    if (split.length == 2) {
      this.type = split[1];
    } else {
      if (input.contains(":")) {
        log.warn("[Symbol()] \":\" in Symbol definition, but nothing following it, type is ambiguous? Input: " + input);
      }
      this.type = "";
    }
  }

  //TODO:brad: this should not be used outside of the Factory
  @Deprecated
  public Symbol(String name, String type) {
    this.name = name;
    this.type = type;
  }

  @Override
  public Symbol clone() {
    try {
      return (Symbol) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError();
    }
  }

  /**
   * Checks if this symbol has arguments.
   * @return always false since Symbol doesn't support arguments
   */
  public boolean hasArgs() {
    return false;
  }

  public boolean isVariable() {
    return false;
  }

  public boolean isConstant() {
    return false;
  }

  public boolean isTerm() {
    return false;
  }

  public boolean isPredicate() {
    return false;
  }
}
