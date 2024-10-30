/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.consultant;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.ArrayList;
import java.util.List;

public class Reference {
  /**
   * Unique reference resolution id (e.g., pose_2).
   */
  final public Symbol refId;
  /**
   * Free-variable in the list of properties is the object of interest.
   */
  final public Variable variable;
  /**
   * List of properties relevant to this reference.
   */
  final public List<Term> properties;

  public Reference(Symbol ref, Variable variable) {
    this.refId = ref;
    this.variable = variable;
    this.properties = new ArrayList<>();
  }

  public Reference(Symbol ref, Variable variable, List<Term> properties) {
    this(ref, variable);
    this.properties.addAll(properties);
  }

  @Deprecated
  public Symbol getRefId() {
    return this.refId;
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append(refId).append(" var = ").append(variable).append(" properties = ").append(properties);
    return sb.toString();
  }
}
