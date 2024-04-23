/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fol.util;

import edu.tufts.hrilab.fol.Constant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Utilities {

  static private Logger log = LoggerFactory.getLogger(Utilities.class);

  /**
   * Transforms action variables to prolog variables
   *
   * @param t term
   * @return modified term
   */
  static public Term actionToProlog(Term t) {
    Term term = new Term(t);
    List<Symbol> args = new ArrayList<>();
    for (Symbol s : term.getArgs()) {
      if (s.isTerm()) {
        args.add(actionToProlog((Term) s));
      } else {
        if (s.getName().startsWith("?")) {
          s.setName("_g" + s.getName().substring(1));
        } else if (s.getName().startsWith("!")) {
          s.setName("_l" + s.getName().substring(1));
        }
        args.add(s);
      }
    }
    return new Term(t.getName(), args);
  }

  /**
   * Transforms prolog variables to action variables
   *
   * @param v variable
   * @return action variable name
   */
  static public Variable prologToAction(Variable v) {
    if (v.getName().startsWith("_g")) {
      return new Variable("?" + v.getName().substring(2));
    } else if (v.getName().startsWith("_l")) {
      return new Variable("!" + v.getName().substring(2));
    } else return v;
  }

  /**
   * Create conjunctive "and" predicate with the list of predicates as the arguments.
   * @param predicates
   * @return
   */
  static public Predicate createAndPredicate(List<Predicate> predicates) {
    if (predicates.isEmpty()) {
      log.warn("[createAndPredicate] received empty list.");
      return Factory.createPredicate("and");
    } else if (predicates.size() > 1) {
      return Factory.createPredicate("and", predicates);
    } else {
      return predicates.get(0);
    }
  }

  /**
   * Checks if value's toString is an integer, long, float, or double.
   *
   * @param value
   * @return
   */
  public static boolean isNumeric(Object value) {
    return value.toString().matches("[-+]?\\d*\\.?\\d+[fFlLdD]?");
  }

  /**
   * Checks if all arguments are typed. If there are nested Terms,
   * only the Term's arguments need to be typed (i.e., not the Term itself).
   * @param term
   * @return
   */
  static public boolean isFullyTyped(Term term) {
    for (Symbol arg : term.getArgs()) {
      if (arg.isTerm()) {
        if (!isFullyTyped((Term)arg)) {
          return false;
        }
      } else if (!arg.hasType() && !isNumeric(arg)) {
        return false;
      }
    }

    return true;
  }

  /**
   * Checks if s1 and s2 are equal, but ignores the type of
   * @param s1
   * @param s2
   * @return
   */
  static public boolean equalsIgnoreType(Symbol s1, Symbol s2) {
    if (s1 == s2) return true;
    if (s1 == null || s2 == null || s1.getClass() != s2.getClass()) return false;
    if (s1.isTerm()) {
      // both s1 and s2 are Term or Predicate
      if (!s1.getName().equals((s2.getName()))) return false;
      Term t1 = (Term) s1;
      Term t2 = (Term) s2;
      if (t1.size() != t2.size()) return false;
      for (int i = 0; i < t1.size(); ++i) {
        if (!equalsIgnoreType(t1.get(i), t2.get(i))) {
          return false;
        }
      }
      return true;
    } else if (s1.isVariable()) {
      // both s1 and s2 are Variables
      Variable v1 = (Variable) s1;
      Variable v2 = (Variable) s2;
      return v1.getValue().equals((v2.getValue())) && s1.getName().equals((s2.getName()));
    } else if (s1.isConstant()) {
      // both s1 and s2 are Constants
      Constant c1 = (Constant) s1;
      Constant c2 = (Constant) s2;
      return c1.getValue().equals((c2.getValue())) && s1.getName().equals((s2.getName()));
    } else {
      // both s1 and s2 are Symbols
      return s1.getName().equals((s2.getName()));
    }
  }

    /**
     * Determine if both lists of Predicates match. Variable names are allowed to
     * mismatch as long as their types and values are the same (or empty/null). Order doesn't matter.
     *
     * @param list1
     * @param list2
     * @return
     */
    static public boolean predicatesMatch(List<? extends Term> list1, List<? extends Term> list2) {
      if (list1.size() != list2.size()) {
        return false;
      }

      return containsAllPredicates(list1, list2);
    }

    /**
     * Determine if two Predicates match. Variable names are allowed to mismatch
     * as long as their types and values are the same (or empty/null).
     *
     * @param t1 Term
     * @param t2 Term
     * @return
     */
    static public boolean predicatesMatch(Term t1, Term t2) {
      //check names
      if (!t1.getName().equalsIgnoreCase(t2.getName())) {
        return false;
      }
      //check size of args
      if (t1.size() != t2.size()) {
        return false;
      }
      //check args -- order matters
      for (int i = 0; i < t1.size(); ++i) {
        Symbol sym1 = t1.get(i);
        Symbol sym2 = t2.get(i);
        //special case for Variable, since names can mismatch under certain conditions
        //should there also be a special case for Constant?
        if (sym1.isTerm() && sym2.isTerm()) {
          if (!predicatesMatch((Term) sym1, (Term) sym2)) {
            return false;
          }
        } else if (sym1.isVariable() && sym2.isVariable()) {
          //names don't have to be equal, just non-empty types (not sure about value?!)
          String type1 = ((Variable) sym1).getType();
          String type2 = ((Variable) sym2).getType();
          if ((type1 != null && !type1.isEmpty())
                  && (type2 != null && !type2.isEmpty())
                  && (!type1.equalsIgnoreCase(type2))) {
            return false;
          }
        } else if (!sym1.equals(sym2)) {
          return false;
        }
      }

      return true;
    }

  /**
   * Check if list1 contains all of list2's Terms.
   *
   * @param list1
   * @param list2
   * @return
   */
  static public boolean containsAllPredicates(List<? extends Term> list1, List<? extends Term> list2) {

    boolean matchFound = true;
    for (Term p2 : list2) {
      matchFound = false;
      for (Term p1 : list1) {
        if (predicatesMatch(p1, p2)) {
          matchFound = true;
          break;
        }
      }
      if (!matchFound) {
        //break from outer loop (which will return false)
        break;
      }
    }

    return matchFound;
  }


  /**
   * Convenience method for {@code getUnboundVariables(Symbol s, Set<Variable> vars)}.
   *
   * @param symbols List of Symbols
   * @return List of unbound Variables
   */
  static public List<Variable> getUnboundVariables(List<? extends Symbol> symbols) {
    Set<Variable> vars = new HashSet<>();
    for (Symbol s : symbols) {
      vars.addAll(getUnboundVariables(s));
    }
    return new ArrayList<>(vars);
  }

  /**
   * Convenience method for {@code getUnboundVariables(Symbol s, Set<Variable> vars)}.
   *
   * @param s Symbol
   * @return Set of unbound Variables
   */
  static public List<Variable> getUnboundVariables(Symbol s) {
    List<Variable> vars = new ArrayList<>();
    getUnboundVariables(s, vars);
    return vars;
  }

  /**
   * Collect all unbound Variables (i.e., with no/null value). This recursively
   * checks through nested Symbols and also through Variables bound to
   * Variables.
   *
   * @param s Symbol
   * @param vars non-null set of Variables
   */
  static public void getUnboundVariables(Symbol s, List<Variable> vars) {
    if (vars == null) {
      log.error("[getUnboundVariables] passed in List is null.");
      return;
    }

    if (s == null) {
      return;
    } else if (s.isVariable()) {
      Variable s_var = (Variable) s;
      if (s_var.getValue() == null && !vars.contains(s_var)) {
        vars.add(s_var);
      } else {
        getUnboundVariables(s_var.getValue(), vars);
      }
    } else if (s instanceof Term) {
      Term s_term = (Term) s;
      for (Symbol arg : s_term.getArgs()) {
        getUnboundVariables(arg, vars);
      }
    }
  }

}
