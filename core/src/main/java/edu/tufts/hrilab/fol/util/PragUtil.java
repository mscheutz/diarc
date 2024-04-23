/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fol.util;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.*;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

// TODO: move relevant method to edu.tufts.hrilab.fol.util.Utilities class.

public class PragUtil {
  private static Logger log = LoggerFactory.getLogger(PragUtil.class);
  ///////////////////////////////////////////////////////////
  // Get bound version methods
  ///////////////////////////////////////////////////////////

  public static Symbol getBoundSymbol(Map<Variable, Symbol> bindings, Symbol sym) {
    if (sym.isVariable()) {
      Symbol tmp = bindings.get(sym);
      if (tmp != null) {
        return getBoundSymbol(bindings, tmp);
      } else {
        return sym;
      }
    } else if (sym.isTerm()) {
      return getBoundTerm(bindings, (Term) sym);
    }
    return sym;
  }

  public static Symbol getBoundSymbolVars(Map<Variable, Symbol> bindings, Symbol sym) {
    if (sym.isVariable()) {
      Variable sv = ((Variable) sym);
      for (Variable v : bindings.keySet()) {
        if (v.getName().equals(sv.getName())
            && (v.getType().equals(sv.getType())
            || v.getType().equals("")
            || sv.getType().equals(""))) {
          return bindings.get(v);
        }
      }
      return sym;
    } else if (sym.isTerm()) {
      return getBoundTermVars(bindings, (Term) sym);
    }
    return sym;
  }

  public static Term getBoundTerm(Map<Variable, Symbol> bindings, Term term) {
    String name = term.getName();
    List<Symbol> args = term.getArgsCopy();

    args.replaceAll(symbol -> getBoundSymbol(bindings, symbol));
    return new Predicate(name, args);
  }

  public static Term getBoundTermVars(Map<Variable, Symbol> bindings, Term term) {
    String name = term.getName();
    List<Symbol> args = term.getArgsCopy();

    args.replaceAll(symbol -> getBoundSymbolVars(bindings, symbol));
    return new Predicate(name, args);
  }

  public static Term fillPredicate(Map<Variable, Symbol> bindings, Term term) {
    String name = term.getName();
    List<Symbol> args = term.getArgsCopy();

    args.replaceAll(symbol -> fillSymbol(bindings, symbol));
    return new Predicate(name, args);
  }

  public static Symbol fillSymbol(Map<Variable, Symbol> bindings, Symbol s) {
    if (s.isVariable()) {
      for (Variable v : bindings.keySet()) {
        if (v.getName().equals(s.getName())) {
          return bindings.get(v);
        }
      }
      return s;
    } else if (s.isTerm()) {
      return fillPredicate(bindings, (Term) s);
    }
    return s;
  }


  ///////////////////////////////////////////////////////////
  // Determine binding methods
  ///////////////////////////////////////////////////////////
  public static boolean termEquals(Term t1, Term t2) {
    return getTermBindings(t1, t2) != null;
  }

  public static boolean compareTermStruct(Term t1, Term t2) {
    return (t1.getName().equals(t2.getName()) && t1.getArgs().size() == t2.getArgs().size());
  }

  public static Map<Variable, Symbol> getSymbolBindings(Symbol s1, Symbol s2) {
    return getBindingsSymHelper(new HashMap<>(), s1, s2);
  }

  public static Map<Variable, Symbol> getBindingsSymHelper(Map<Variable, Symbol> bindings, Symbol s1, Symbol s2) {
    if (bindings == null) {
      log.warn("[getBindingsSymHelper] passed in null bindings. Returning null. s1: "+s1+" s2: "+s2);
      return null;
    }

    if (s1.isTerm() && s2.isTerm()) {
      return getBindingsTermHelper(bindings, (Term) s1, (Term) s2);
    } else if (s1.isVariable() && s2 != null) { //Brad: replaced check for instanceof symbol with != null
      Symbol tmp = bindings.get(s1);
      if (tmp == null) {
        bindings.put((Variable) s1, s2);
      } else if (!tmp.equals(s2)) {
        return null;
      }

    } else if (s1.getName().equals("_")) {
      return bindings;
    } else if (!Utilities.equalsIgnoreType(s1, s2)) {
      return null;
    }
    return bindings;
  }

  public static Map<Variable, Symbol> getTermBindings(Term t1, Term t2) {
    return getBindingsTermHelper(new HashMap<>(), t1, t2);
  }

  public static Map<Variable, Symbol> getTermBindingsVars(Term t1, Term t2) {
    Map<Variable, Symbol> bindings = new HashMap<>();
    if (!compareTermStruct(t1, t2)) {
      return null;
    }

    List<Symbol> t1sym = t1.getArgs();
    List<Symbol> t2sym = t2.getArgs();

    for (int i = 0; i < t1sym.size(); i++) {
      Symbol s1 = t1sym.get(i);
      Symbol s2 = t2sym.get(i);
      if (s1.isTerm() && s2.isTerm()) {
        Map<Variable, Symbol> newBindings = getBindingsTermHelper(bindings, (Term) s1, (Term) s2);
        if (newBindings != null) {
          bindings = newBindings;
        } else {
          return null;
        }
      } else if (s1.isVariable() && s2.isVariable()) {

        if (!(s1.getType().equals(s2.getType())
            || (s1.getType().equals(""))
            || (s2.getType().equals("")))) {
          return null;
        }
      } else if (s1.isVariable() && s2 != null) {
        Symbol tmp = bindings.get(s1);
        if (tmp == null) {
          bindings.put((Variable) s1, s2);
        } else if (tmp.isTerm() && s2.isTerm()) {
          Map<Variable, Symbol> newBindings = getBindingsTermHelper(bindings, (Term) tmp, (Term) s2);
          if (newBindings != null) {
            bindings = newBindings;
          } else {
            return null;
          }
        } else if (!tmp.equals(s2)) {
          return null;
        }
      } else {
        try {
          assert s2 != null;
          if (!namesMatch(s1.getName(), s2.getName())) {
            return null;
          }
        } catch (Exception e) {
          log.error("Exception caught.", e);
        }
      }
    }
    return bindings;
  }

  public static Map<Variable, Symbol> getBindingsTermHelper(Map<Variable, Symbol> bindings, Term t1, Term t2) {
    if (bindings == null) {
      log.warn("[getBindingsTermHelper] passed in null bindings. Returning null.");
      return null;
    }
    if (!compareTermStruct(t1, t2)) {
      return null;
    }

    List<Symbol> t1sym = t1.getArgs();
    List<Symbol> t2sym = t2.getArgs();

    for (int i = 0; i < t1sym.size(); i++) {
      Symbol s1 = t1sym.get(i);
      Symbol s2 = t2sym.get(i);
      if (s1.isTerm() && s2.isTerm()) {
        Map<Variable, Symbol> newBindings = getBindingsTermHelper(bindings, (Term) s1, (Term) s2);
        if (newBindings != null) {
          //TODO:brad: does this need to happen?
          bindings = newBindings;
        } else {
          return null;
        }
      } else if (s1.isVariable() && s2 != null) { //brad: replace instanceof Symbol check with null check
        Symbol tmp = bindings.get(s1);
        if (tmp == null) {
          bindings.put((Variable) s1, s2);
        } else if (tmp.isTerm() && s2.isTerm()) {
          Map<Variable, Symbol> newBindings = getBindingsTermHelper(bindings, (Term) tmp, (Term) s2);
          if (newBindings != null) {
            bindings = newBindings;
          } else {
            return null;
          }
        } else if (!tmp.equals(s2)) {
          return null;
        }
      } else {
        // s1 is a Symbol, ignore s2's type if it has one
        if (!Utilities.equalsIgnoreType(s1, s2)) {
          return null;
        }
      }
    }
    return bindings;
  }

  //------------------------------------------------
  //------------------------------------------------

  public static boolean isLong(String str) {
    try {
      Long.parseLong(str);
    } catch (Exception e) {
      return false;
    }
    return true;
  }

  private static boolean namesMatch(String s1, String s2) {
    boolean s1ref = s1.contains("_") && isLong(s1.split("_")[1]);
    boolean s2ref = s1.contains("_") && isLong(s1.split("_")[1]);

    if (s1ref == s2ref) {
      return (s1.equals(s2));
    }
    if (s1ref) {
      return s1.split("_")[1].equals(s2);
    }
    /*if(s2ref) (always true) */
    return s2.split("_")[1].equals(s1);
  }

  //------------------------------------------------
  //------------------------------------------------

  public static Term formPrologQueryTerm(Term t) {
    Set<Variable> varSet = t.getVars();
    Map<Variable, Symbol> binding = new HashMap<>();
    for (Variable v : varSet) {
      binding.put(v, new Symbol(v.getName()));
    }
    return getBoundTerm(binding, t);
  }

  //splits s on delim, ignoring instances of pattern within ()
  //have to do it like this because java doesn't support recursive regexs
  public static List<String> splitToplevel(String s, char delim) {
    List<String> results = new ArrayList<>();
    int parenDepth = 0;
    int resultIdx = 0;

    results.add("");
    for (char c : s.toCharArray()) {
      if (c == '(') parenDepth++;
      else if (c == ')') parenDepth--;

      if (c == delim && parenDepth == 0) {
        resultIdx++;
        results.add("");
      } else {
        results.set(resultIdx, results.get(resultIdx) + c);
      }
    }
    return results;
  }


  public static List<Term> parseModifiers(String line) {
    List<Term> retList = new ArrayList<>();
    int opar = line.indexOf('{');
    int cpar = line.lastIndexOf('}');

    // for (no set) single variable case (e.g., "Z" instead of "{...}") 
    if (opar < 0) {
      return null;
    }

    String modStr = line.substring(opar + 1, cpar);
    List<String> modifiers = splitToplevel(modStr,',');

    for (String s : modifiers) {
      s = s.trim();
      if (s.isEmpty()) {
        continue;
      }
      if (s.indexOf('(') > 0) {
        retList.add( Factory.createPredicate(s));
      }
    }

    return retList;
  }

}
