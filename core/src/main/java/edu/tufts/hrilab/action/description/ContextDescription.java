/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.description;

import edu.tufts.hrilab.fol.Predicate;

import java.util.List;

public abstract class ContextDescription {

  public abstract List<Predicate> getPredicateDescription();
  public abstract Predicate getStepsInPredicateForm();

}
