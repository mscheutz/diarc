/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.operators;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;
import java.util.Set;

public final class FOL {

  /**
   * Symbol operators
   */

  public static String getName(Symbol symbol) {
    return symbol.getName();
  }

  /**
   * Term operators
   */

  public static int size(Term term){
    return term.size();
  }

  //TODO:brad: I changed the name of this because of a collision with the get operation in Collections.java. This change covers up the fact that we can't correctly differentiate between the two at runtime. See the additional notes in Database.java and isAssignableFrom in Utilities.java
  public static Symbol getArg(Term term, int index){
    return term.get(index);
  }

  public static Set<Variable> getVars(Term term){
    return term.getVars();
  }

  public static List<Symbol> getArgs(Term term){
    return term.getArgs();
  }

  /**
   * Utility
   */

  // TODO:brad: add all FOL factory constructors, or remove this one
  public static Predicate createPredicate(String desc){
    return Factory.createPredicate(desc);
  }

}