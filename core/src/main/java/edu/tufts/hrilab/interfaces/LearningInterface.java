/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Term;

//TODO:brad: I think this should be combined with the consultant interface
/**
 * LearningComponent is the base set of methods for learning/unlearning concepts.
 */
public interface LearningInterface {

  /**
   * Use the passed in term to learn a new descriptor. The term must have two
   * arguments, the first arg 'describing' the new thing-to-learn, and the
   * second arg 'naming' the new thing-to-learn. The description must be
   * composed entirely of things vision already knows about.
   *
   * @param learningTerm term specifying the thing to learn (e.g., instanceOf(X,Y))
   * @return if thing-to-learn was successfully learned
   */
  @TRADEService
  @Action
  Justification learn(Term learningTerm);

  /**
   * Un-learns things that have been learned by the learn method.
   * @param learningTerm
   * @return
   */
  @TRADEService
  @Action
  Justification unlearn(Term learningTerm);
}


