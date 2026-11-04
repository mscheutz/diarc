/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.area;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;

public class ABBAreaReference extends Reference {

  public ABBAreaReference(Symbol ref, Variable variable, List<Term> properties) {
    super(ref, variable, properties);
  }

  public ABBAreaReference(Symbol ref, Variable variable) {
    super(ref, variable);
  }
}
