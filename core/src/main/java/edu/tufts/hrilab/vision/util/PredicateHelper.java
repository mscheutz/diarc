/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.util;

import edu.tufts.hrilab.consultant.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Helper methods to deal with Predicates in VisionComponent.
 *
 * @author Evan Krause
 */
public class PredicateHelper {

  static Logger log = LoggerFactory.getLogger(PredicateHelper.class);

  private static int variableIndex = 0;

  // TODO: get this value from constultant KB name
  public static final String varType = "physobj";

  /**
   * Get the representative string from a Predicate. This is pretty much a hack
   * in order to pass the appropriate string to the native side and to look up
   * the associated VisionProcess via the hashmaps they are stored in. This can
   * hopefully be made more general and non-hacky once Predicates have been
   * standardized in DIARC, and if Predicates can be handled directly on the
   * native side. Vision assumes all predicates have descriptor as functor name.
   * Examples: red(X) -> red, near(A,B) -> near
   *
   * @param term
   * @return
   */
  static public String getRepresentativeString(Term term) {
    return term.getName();
  }

  /**
   * Basically a customized toString method for the Symbol class hierarchy. Does
   * not retain Variable type, and also converts bound Variables to their bound
   * value, disregarding the Variable name and type (e.g., name/binding =
   * A/red(X) --> red(X)).
   *
   * @param symbol
   * @return
   */
  static public String getNativeStringRepresentation(Symbol symbol) {
    StringBuilder sb = new StringBuilder();
    if (symbol.isVariable()) {
      Symbol binding = ((Variable) symbol).getValue();
      if (binding == null) {
        sb.append(symbol.getName());
      } else {
        sb.append(getNativeStringRepresentation(binding));
      }
    } else if (symbol.isTerm()) {
      sb.append(symbol.getName()).append("(");
      String sp = "";
      for (Symbol arg : ((Term) symbol).getArgs()) {
        sb.append(sp).append(getNativeStringRepresentation(arg));
        sp = ",";
      }
      sb.append(")");
    } else {
      // either a pure Symbol or Constant
      sb.append(symbol.toString());
    }

    return sb.toString();
  }

  static public Map<List<Variable>, List<Term>> splitIntoEntities(List<? extends Term> predicates) {
    Map<List<Variable>, List<Term>> entities = new LinkedHashMap<>();

    List<Variable> vars;
    List<Term> preds;
    for (Term d : predicates) {
      vars = edu.tufts.hrilab.fol.util.Utilities.getUnboundVariables(d);
      preds = entities.get(vars);
      if (preds == null) {
        preds = new ArrayList();
        entities.put(vars, preds);
      }
      preds.add(d);
    }

    return entities;
  }

  static public List<Term> convertToVisionForm(List<? extends Symbol> symbols) {
    List<Term> visionDescriptors = PredicateHelper.insertVariables(symbols);
    visionDescriptors = PredicateHelper.unpackAndFlatten(visionDescriptors);
    removeDuplicates(visionDescriptors);
    return visionDescriptors;
  }

  static public List<Term> convertToVisionForm(Symbol symbol) {
    Term visionDescriptor = PredicateHelper.insertVariables(symbol);
    if (visionDescriptor == null) {
      log.warn("[convertToVisionForm] can't convert to list of Terms: " + symbol);
      return null;
    } else {
      return PredicateHelper.unpackAndFlatten(visionDescriptor);
    }
  }

  static public void removeDuplicates(List<? extends Symbol> symbols) {
    Set<Symbol> symbolSet = new HashSet<>();
    Iterator<? extends Symbol> symbols_itr = symbols.iterator();
    while (symbols_itr.hasNext()) {
      Symbol symbol = symbols_itr.next();
      if (symbolSet.contains(symbol)) {
        symbols_itr.remove();
      } else {
        symbolSet.add(symbol);
      }

    }
  }

  /**
   * Take list of Symbols and insert Variables into appropriate places to
   * construct a valid visual search request. Currently, in vision, all Terms
   * should "bottom out" at a Variable for each of its arguments, except in the
   * case of object refs (e.g., object_3).
   * For example:
   *   knife --> knife(X);
   *   on(grasp_point,handle) --> on(grasp_point(X),handle(Y));
   *   on(grasp_point,object_4) --> on(grasp_point(X),object_4);
   * NOTE: There's currently no mechanism to insert the same Variable into more
   * than one place, as it's not clear if that's ever needed or how it would
   * work.
   *
   * @param symbols
   * @return
   */
  static private List<Term> insertVariables(List<? extends Symbol> symbols) {

    // find what variables, if any, are already being used so they aren't duplicated during insertion
    Set<Variable> usedVars = new HashSet<>();
    for (Symbol symbol : symbols) {
      if (symbol.isTerm()) {
        usedVars.addAll(((Term) symbol).getVars());
      }
    }

    // now do variable insertion
    List<Term> variablizedTerms = new ArrayList<>();
    for (Symbol s : symbols) {
      Symbol variablizedSymbol = insertVariablesHelper(s, usedVars);
      if (variablizedSymbol.isTerm()) {
        variablizedTerms.add((Term) variablizedSymbol);
      }
    }

    return variablizedTerms;
  }

  /**
   * A convenience method to the insertVariables method that takes in a single Symbol.
   *
   * @param symbol
   * @return
   */
  static private Term insertVariables(Symbol symbol) {
    // find what variables, if any, are already being used so they aren't duplicated during insertion
    Set<Variable> usedVars = new HashSet<>();
    if (symbol.isTerm()) {
      usedVars = ((Term) symbol).getVars();
    }

    Symbol variablizedSymbol = insertVariablesHelper(symbol, usedVars);
    if (variablizedSymbol.isTerm()) {
      return (Term) variablizedSymbol;
    } else {
      log.error("[insertVariables] could not insert variables into: " + symbol);
      return null;
    }
  }

  /**
   * Helper method to the other insertVariables methods to do the recursive
   * variable insertion.
   *
   * @param symbol
   * @param usedVars variables already in use in the passed in symbols
   * @return
   */
  static private Symbol insertVariablesHelper(Symbol symbol, Set<Variable> usedVars) {

    List<Symbol> variablizedArgs = new ArrayList<>();
    if (symbol.isVariable()) {
      // if already a variable, just return it
      return symbol;
    } else if (symbol.isTerm()) {
      Term s_term = (Term) symbol;
      for (Symbol arg : s_term.getArgs()) {
        variablizedArgs.add(insertVariablesHelper(arg, usedVars));
      }
    } else if (isObjectRef(symbol)) {
      // plain old symbol that is an object ref
      return symbol;
    } else {
      // just a plain old Symbol, not a sub-type -- add a Variable !!!
      Variable var = new Variable("VAR"+variableIndex++, PredicateHelper.varType);
      while (usedVars.contains(var)) {
        var = new Variable("VAR"+variableIndex++, PredicateHelper.varType);
      }
      variablizedArgs.add(var);
    }

    return new Term(symbol.getName(), variablizedArgs);
  }

  static private List<Term> unpackAndFlatten(Term term) {
    List<Term> unpackedTerms = unpack(term);
    return flatten(unpackedTerms);
  }

  static private List<Term> unpackAndFlatten(List<? extends Term> terms) {
    List<Term> unpackedTerms = unpack(terms);
    return flatten(unpackedTerms);
  }

  /**
   * Turn already unpacked Terms into flattened (un-nested) Terms. For example,
   * the following unpacked list of terms: [grasp_point(A), handle(B), knife(C),
   * part_of(handle(B),knife(C)),on(grasp_point(A),part_of(handle(B),knife(C)))]
   * will get flattened to : [grasp_point(A), handle(B), knife(C), part_of(B,C),
   * on(A,B)].
   *
   * NOTE: this makes a lot of assumptions about the order of Terms in the list
   * i.e., that inner nested terms appear in the list before the term they are
   * nested in. For example, [handle(B),knife(C),part_of(handle(B),knife(C))]
   * and not [part_of(handle(B),knife(C)),handle(B),knife(C)].
   *
   * @param terms
   * @return
   */
  static private List<Term> flatten(List<? extends Term> terms) {
    List<Term> flattened = new ArrayList<>();

    // mappings from Term to un-nested Symbol replacement
    Map<Symbol, Symbol> mappings = new HashMap<>();

    // iterator through unpacked list, un-nesting Terms as we go
    for (Term t : terms) {
      List<Symbol> newArgs = new ArrayList<>();
      for (Symbol s : t.getArgs()) {
        if (s.isTerm()) {
          // replace nested Term with single un-nested Symbol
          if (mappings.containsKey(s)) {
            newArgs.add(mappings.get(s));
          } else {
            log.error("[flatten] could not replace nested Term: " + s);
            newArgs.add(s);
          }
        } else {
          newArgs.add(s);
        }
      }

      // add (old) Term to the mappings
      if (newArgs.size() > 0) {
        mappings.put(t, newArgs.get(0));
      } else {
        log.error("[flatten] encountered Term with arity zero: " + t);
      }

      // construct new un-nested Term and add it to the return list
      Term newTerm = new Term(t.getName(), newArgs);
      flattened.add(newTerm);
    }

    return flattened;
  }

  /**
   * Recursively searches through symbols and adds all Predicates to output
   * list. This will separately add Terms, nested Terms within Terms, and also
   * Terms bound to Variables. Critically, nested Predicates will be added to
   * output list before the containing Predicate. This ensures nested Predicates
   * are processed first.
   *
   * @param symbols to flatten
   * @return
   */
  static private List<Term> unpack(List<? extends Symbol> symbols) {
    List<Term> unpacked = new ArrayList<>();
    for (Symbol s : symbols) {
      unpack(s, unpacked);
    }
    return unpacked;
  }

  /**
   * Recursively searches through symbols and adds all Predicates to output
   * list. This will separately add Terms, nested Terms within Terms, and also
   * Terms bound to Variables. Critically, nested Predicates will be added to
   * output list before the containing Predicate. This ensures nested Predicates
   * are processed first.
   *
   * @param symbol to flatten
   * @return
   */
  static private List<Term> unpack(Symbol symbol) {
    List<Term> unpacked = new ArrayList<>();
    unpack(symbol, unpacked);
    return unpacked;
  }

  /**
   * Recursively searches through the Symbol and adds all Terms to flattened
   * list. This will separately add Terms, nested Terms within Terms, and also
   * Terms bound to Variables. Critically, nested Predicates will be added to
   * output list before the containing Term. This ensures nested Terms are
   * processed first.
   *
   * @param s symbol to unpack
   * @param unpacked
   */
  static private void unpack(Symbol s, List<Term> unpacked) {
    if (unpacked == null) {
      log.error("[unpack] passed in List is null.");
      return;
    }

    if (s.isTerm()) {
      Term s_term = (Term) s;
      for (Symbol arg : s_term.getArgs()) {
        unpack(arg, unpacked);
      }
      unpacked.add(s_term); // add current term *after* nested term
    } else if (s.isVariable()) {
      Symbol binding = ((Variable) s).getValue();
      if (binding != null) {
        unpack(binding, unpacked);
      }
    }
  }

  /**
   * Order the variables contained in the list of terms using the relation terms
   * (i.e., Term with 2 variables) as the ordering constraints where the second
   * variable of a relation needs to come first in the ordering. For example,
   * handle(Y), on(Y,X), mug(X), would produce the ordering X -> Y.
   *
   * @param terms
   * @return
   */
  static public List<Variable> orderVariables(List<? extends Term> terms) {
    // separate into single variables and variable pairs (constraints)
    List<Variable> vars = new ArrayList<>();
    List<List<Variable>> constraints = new ArrayList<>();
    List<Variable> currVars;
    for (Term term : terms) {
      currVars = term.getOrderedVars();
      if (currVars.size() == 1) {
        if (!vars.contains(currVars.get(0))) {
          vars.add(currVars.get(0));
        }
      } else if (currVars.size() == 2) {
        if (!constraints.contains(currVars)) {
          constraints.add(currVars);
          // also add individual vars from constraint
          for (Variable v : currVars) {
            if (!vars.contains(v)) {
              vars.add(v);
            }
          }
        }
      } else {
        log.error("[orderVariables] can't handle constraints with more than 2 Variablse.");
      }
    }

    // proper ordering of Variables
    // solve CSP with single variables as values and
    // variable pairs as ordering constraints 
    // e.g., (X,Y) means Y should be before X
    List<Variable> orderedVars = new ArrayList<>();
    orderVariablesBacktrack(orderedVars, vars, constraints);
    return orderedVars;
  }

  /**
   * Order the set of variable lists using relations (i.e., list with 2
   * variables) as the ordering constraints where the second variable of a
   * relation needs to come first in the ordering. For example, {@code handle(Y) &
   * on(Y,X) & mug(X)}, which produces the following lists of variables
   * [Y],[Y,X],[X] would produce the ordering X -> Y -> X,Y.
   *
   * @param unorderedVars
   * @return
   */
  static public List<List<Variable>> orderVariableLists(Set<List<Variable>> unorderedVars) {
    // separate into single variables and variable pairs (constraints)
    List<Variable> vars = new ArrayList<>();
    List<List<Variable>> constraints = new ArrayList<>();
    Iterator<List<Variable>> itr = unorderedVars.iterator();
    List<Variable> currVars;
    while (itr.hasNext()) {
      currVars = itr.next();
      if (currVars.size() == 1) {
        if (!vars.contains(currVars.get(0))) {
          vars.add(currVars.get(0));
        }
      } else if (currVars.size() == 2) {
        if (!constraints.contains(currVars)) {
          constraints.add(currVars);
          // also add individual vars from constraint
          for (Variable v : currVars) {
            if (!vars.contains(v)) {
              vars.add(v);
            }
          }
        }
      } else {
        log.error("[orderVariables] can't handle constraints with more than 2 Variables.");
      }
    }

    // proper ordering of Variables
    // solve CSP with single variables as values and
    // variable pairs as ordering constraints 
    // e.g., (X,Y) means Y should be before X
    List<Variable> orderedVars = new ArrayList<>();
    orderVariablesBacktrack(orderedVars, vars, constraints);

    // for now just add the single vars first, then the pairs
    List<List<Variable>> orderedVarLists = new ArrayList<>();
    List<Variable> tmpVars;
    for (Variable v : orderedVars) {
      tmpVars = new ArrayList<>();
      tmpVars.add(v);
      orderedVarLists.add(tmpVars);
    }
    for (List<Variable> constraint : constraints) {
      orderedVarLists.add(constraint);
    }

    return orderedVarLists;
  }

  /**
   * Recursive CSP to order variables.
   *
   * @param orderedVars return list to fill
   * @param values variables to put in order
   * @param constraints variable pairs (second variable in pair needs to come
   * first)
   */
  static private boolean orderVariablesBacktrack(List<Variable> orderedVars, List<Variable> values, List<List<Variable>> constraints) {
    // completion check
    if (orderedVars.size() == values.size()) {
      return true;
    }

    // find next index to fill (unassigned variable)
    int i = orderedVars.size();

    // try all remaining assignments of index i
    for (Variable currValue : values) {
      if (orderedVars.contains(currValue)) {
        continue;
      }

      // tenatively add new assignment
      orderedVars.add(i, currValue);

      // check current assignment against all constraints
      boolean consistent = true;
      for (List<Variable> constraint : constraints) {
        for (int j = 0; j < constraint.size() - 1; ++j) {
          int first = orderedVars.indexOf(constraint.get(j + 1));
          int second = orderedVars.indexOf(constraint.get(j));
          if (first != -1 && second != -1 && first > second) {
            consistent = false;
          }
        }
      }
      if (consistent) {
        if (orderVariablesBacktrack(orderedVars, values, constraints)) {
          return true;
        } else {
          orderedVars.remove(i);
        }
      } else {
        orderedVars.remove(i);
      }
    }

    // failure
    return false;
  }

  /**
   * Replace all occurences of argOld with argNew in term and return a
   * new Predicate with the specified replacements.
   * @param term
   * @param argOld
   * @param argNew
   * @return
   */
  public static Predicate replace(Term term, Symbol argOld, Symbol argNew) {
    List<Symbol> newArgs = new ArrayList<>();
    for (Symbol arg : term.getArgs()) {
      if (arg.equals(argOld)) {
        newArgs.add(argNew);
      } else if (arg.isTerm()) {
        //recurse
        newArgs.add(replace((Term) arg, argOld, argNew));
      } else {
        // keep existing arg
        newArgs.add(arg);
      }
    }

    return new Predicate(term.getName(), newArgs);
  }

  /**
   * Sets the 'type' field of every Variable contained in the symbol.
   * @param symbol
   * @param type
   */
  public static void setVariableType(Symbol symbol, String type) {
    if (symbol.isVariable()) {
      // if a variable, set it's type
      Variable s_var = (Variable) symbol;
      s_var.setType(type);
    } else if (symbol.isTerm()) {
      Term s_term = (Term) symbol;
      for (Symbol arg : s_term.getArgs()) {
        setVariableType(arg, type);
      }
    }
  }

  /**
   * Wrapper around Factory.createPredicate that sets all variables
   * to varType (e.g., "object").
   * @param s
   * @return
   */
  public static Predicate createPredicate(String s) {
    Predicate p = Factory.createPredicate(s);
    setVariableType(p, varType);
    return p;
  }

  //brad: this lets us pass in the KBName instead of having things hard coded to object
  /**
   * Wrapper around Util.createPredicate that sets all variables
   * to varType (e.g., type).
   * @param s
   * @return
   */
  public static Predicate createPredicate(String s, String type) {
    Predicate p = Factory.createPredicate(s);
    setVariableType(p, type);
    return p;
  }

  /**
   * Check if a list a descriptors is specifying a POWER object ref. Basically, that
   * there's a single descriptor that matches "[kbName]_[number]" (e.g., object_3).
   * @param descriptors
   * @return
   */
  public static boolean isObjectRef(List<? extends Symbol> descriptors) {
    if (descriptors.size() == 1 && isObjectRef(descriptors.get(0))) {
      return true;
    }
    return false;
  }

  /**
   * Check if a single descriptor is a POWER object ref that
   * matches "[kbName]_[number]" (e.g., object_3).
   * @param descriptor
   * @return
   */
  public static boolean isObjectRef(Symbol descriptor) {
    return Utilities.isReference(descriptor, varType);
  }
}
