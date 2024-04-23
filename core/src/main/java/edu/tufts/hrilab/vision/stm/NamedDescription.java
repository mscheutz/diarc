/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.fol.Term;

import java.util.ArrayList;
import java.util.List;

/**
 * In lieu of a Pair class, this class is to store a name and description
 * pair. For example, medkit(X) -> white(X),box(X),red(Y),cross(Y),on(Y,X).
 */
public class NamedDescription {

  private Term name;
  private List<Term> descriptors;

  public NamedDescription(Term name, List<? extends Term> descriptors) {
    this.name = name;
    this.descriptors = new ArrayList<>(descriptors);  //shallow copy
  }

  public Term getName() {
    return name;
  }

  public List<Term> getDescriptors() {
    return new ArrayList<>(descriptors);
  }

}
