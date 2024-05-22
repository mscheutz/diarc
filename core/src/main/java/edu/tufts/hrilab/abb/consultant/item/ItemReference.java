/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb.consultant.item;

import edu.tufts.hrilab.consultant.Reference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;

public class ItemReference extends Reference {
  //TODO:brad does this ground out to anything
  public ItemReference(Symbol ref, Variable variable) {
    super(ref, variable);
  }

  public ItemReference(Symbol ref, Variable variable, List<Term> properties) {
    super(ref, variable, properties);
  }
}
