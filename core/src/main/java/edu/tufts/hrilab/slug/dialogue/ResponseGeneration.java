/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.dialogue;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.*;

import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class ResponseGeneration {
  private static Logger log = LoggerFactory.getLogger(ResponseGeneration.class);

  /**
   * Create an itk predicate from an ambiguous semantics predicate.
   * Expecting semantics predicate of the form "semantics(boundOption1, boundOption2, ...)"
   * and generating an itk predicate of the form "itk(actor, mean(speaker, or(boundOption1, boundOption2)))"
   *
   * @param actor     single actor
   * @param semantics "semantics(boundOption1, boundOption2, ...)"
   * @return Predicate of the form "itk(actor, mean(speaker, or(boundOption1, boundOption2)))"
   */
  public static Predicate generateClarificationMultipleMeanings(Symbol actor, Symbol speaker, Term semantics) {

    Predicate orPred = Factory.createPredicate("or", semantics.getArgs());
    log.debug("conflict set: " + orPred);
    Predicate meanPred = Factory.createPredicate("mean", speaker, orPred);
    return new Predicate("itk", actor, meanPred);
  }

  /**
   * Create a Predicate representing the failure semantics in the case of unbound free-variable(s) in
   * the supplemental semantics of an Utterance.
   *
   * @param actor
   * @param suppSemantics "suppSemantics(semantics(...), semantics()...)"
   * @return Predicate of form "not(know(actor, meaningOf(...)))"
   */
  public static Predicate generateFailureUnknownMeaning(Symbol actor, Term suppSemantics) {

    // suppSemantics(semantics(...), semantics()...)
    List<Symbol> varDescriptors = new ArrayList<>();
    for (Variable var : suppSemantics.getVars()) {
      Term firstBindingOption = (Term) suppSemantics.get(0);
      for (Symbol suppArg : firstBindingOption.getArgs()) {
        Term suppArg_term = (Term) suppArg;
        if (suppArg_term.getVars().contains(var)) {
          varDescriptors.add(new Symbol(suppArg_term.getName()));
        }
      }
    }

    //apply binding map of variables with symbols
    Symbol p;
    if (varDescriptors.size() > 1) {
      p = Factory.createPredicate("or", varDescriptors);
    } else {
      p = varDescriptors.get(0);
    }

    // not(know(actor, meaningOf(...)))
    return Factory.createPredicate("not", Factory.createPredicate("know", actor, Factory.createPredicate("meaningOf", p)));
  }

}
