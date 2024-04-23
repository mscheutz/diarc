/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fol;

import edu.tufts.hrilab.util.Util;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Factory {

  protected final static Logger log = LoggerFactory.getLogger(Factory.class);

  /**
   * Creates first order logic class based on string description, should be used in cases where it is unknown
   * which FOL class the description string will produce.
   *
   * @param desc
   * @return
   */
  public static Symbol createFOL(String desc) {
    if (desc == null || desc.isEmpty()) {
      return null;
    }

    if (desc.startsWith("\"") && desc.endsWith("\"")) {
      return createSymbol(desc);
    }

    //TODO:brad: what is that quote check for?
    if (!desc.contains(",") && !desc.contains(")")) {
      if (Character.isUpperCase(desc.charAt(0)) || desc.startsWith("?") || desc.startsWith("!")) {
        return new Variable(desc);
      } else {
        return createSymbol(desc);
      }
    }

    return createPredicate(desc);
  }



  /**
   * Creates a Symbol from a string based representation, this supports the syntax of "name:type"
   *
   * @param desc sting specify the name and optionally type of the Symbol to be created.
   * @return
   */
  public static Symbol createSymbol(String desc) {

    if (desc == null || desc.isEmpty()) {
      return null;
    }

    String name="";
    String type="";

    //EW: is called by makePrologSafe, which can pass in arbitrary strings. We cannot split
    //      on : here without checking whether it is escaped first.
    //Not sure if this will/should ever be called without either all or none of input being escaped - handling case
    //  of "n@meWith:SpecialCharacters":type anyway though
    //If malformed input reaches here, we have already messed up somewhere upstream
    int escapeStart = -1;
    int escapeEnd = -1;
    for (int i=0; i<desc.length();i++) {
      if (desc.charAt(i) == '\"') {
        if (escapeStart == -1) {
          escapeStart = i;
        } else {
          escapeEnd = i;
        }
      }
    }

    List<String> split = new ArrayList<>();
    //No quotes, split on ':'
    if (escapeStart == -1) {
      split = Arrays.asList(desc.split(":"));
    } else {
      //Looks like this constructor gets called in more cases than I thought, though looks like old implementation
      //  would have incorrectly split the name into a name/type as well
      //Single ", something wrong upstream
      if (escapeEnd == -1) {
        log.warn("Problematic call to Symbol constructor: " + desc);
        name = "\"" + desc + "\"";
        type = "";
        return new Symbol(name,type);
      }
      //Split only on : outside escaped substring
      int tokenStart = 0;
      for (int i = 0; i < desc.length(); i++) {
        if (desc.charAt(i) == ':' && i > escapeEnd) {
          split.add(desc.substring(tokenStart, i));
          tokenStart = i + 1;
        }
      }
      split.add(desc.substring(tokenStart));
    }

    name = split.get(0);
    if (split.size() == 2) {
      type = split.get(1);
    } else {
      if (split.size() > 2) {
        log.warn("[createSymbol]: in symbol definition, but nothing following it, type is ambiguous? " + desc);
      }
      type = "";
    }
    return new Symbol(name, type);

  }

  /**
   * Creates a Symbol with explicit name and type
   *
   * @param name name of the Symbol to be created
   * @param type type of the Symbol to be created
   * @return
   */
  public static Symbol createSymbol(String name, String type) {
    return new Symbol(name, type);
  }

  public static Variable createVariable(String desc) {
    if (desc == null || desc.isEmpty()) {
      return null;
    }

    if (!desc.contains(" ") && !desc.contains(",") && !desc.contains(")")) {
      if (Character.isUpperCase(desc.charAt(0)) || desc.startsWith("?") || desc.startsWith("!")) {
        return new Variable(desc);
      }
    }

    log.error("Cannot create Variable from: " + desc);
    return null;
  }

  public static Variable createVariable(String name, String type) {
    if (name == null || name.isEmpty()) {
      return null;
    }

    if (!name.contains(" ") && !name.contains(",") && !name.contains(")")) {
      if (Character.isUpperCase(name.charAt(0)) || name.startsWith("?") || name.startsWith("!")) {
        return new Variable(name,type);
      }
    }

    log.error("Cannot create Variable from: " + name);
    return null;
  }

  //Brad: space delimited style is no longer valid
  // function-style description (at(cramer,breakroom))
  public static Predicate createPredicate(String desc) {

    if (desc.contains(")")) {
      return createPredVars(desc);
    } else {
      //TODO:brad: Ideally this should just create a Symbol?
      //hopefully that doesn't happen anywhere, though...
      // EAK: why shouldn't this case just create a predicate with no args?
      //brad: I think the syntax for creating a nullary predicate should include (). I guess if you're explicitly calling createPredicate, it could be okay, but what is the use case for that?
      log.debug("[createPredicate] creating Predicate from string that doesn't contain ) , "+desc+" , did you want to use Factory.createFOL instead? ");
      return new Predicate(desc, new ArrayList<>());
    }
  }

  // create a Predicate
  public static Predicate createPredicate(String name, String... args) {
    List<Symbol> symargs = new ArrayList<>();
    for (String a : args) {
      symargs.add(createPredicateArg(a));
    }
    return new Predicate(name, symargs);
  }

  // create a Predicate
  public static Predicate createPredicate(String name, Symbol... args) {
    List<Symbol> symargs = new ArrayList<>(Arrays.asList(args));
    return new Predicate(name, symargs);
  }

  public static Predicate createPredicate(String name, List<? extends Symbol> args) {
    return new Predicate(name, args);
  }

  /**
   * Convenience method to construct a negated predicate (i.e., wrap input in "not" predicate).
   *
   * @param name
   * @param args
   * @return
   */
  public static Predicate createNegatedPredicate(String name, Symbol... args) {
    Predicate predicate = createPredicate(name, args);
    return createNegatedPredicate(predicate);
  }

  /**
   * Convenience method to construct a negated predicate (i.e., wrap input in "not" predicate).
   *
   * @param name
   * @param args
   * @return
   */
  public static Predicate createNegatedPredicate(String name, List<? extends Symbol> args) {
    Predicate predicate = createPredicate(name, args);
    return createNegatedPredicate(predicate);
  }

  /**
   * Create a negated predicate based on the input predicate. If the input
   * predicate is not negated (i.e., "not" as functor name), then the input predicate
   * is wrapped in a "not" predicate. If the input predicate is already negated, this method
   * will remove the negation.
   *
   * @param predicate
   * @return
   */
  public static Predicate createNegatedPredicate(Predicate predicate) {
    if (predicate.getName().equals("not") && predicate.size() == 1) {
      if(predicate.get(0).isPredicate()) {
        return new Predicate((Predicate) predicate.get(0));
      } else{
        log.warn("[createNegatedPredicate] trying to negate something that isn't a predicate: "+predicate);
        //TODO:brad: not exactly sure what to do here...
        return new Predicate(predicate.get(0).getName(),Factory.createSymbol("",""));
      }
    } else {
      return new Predicate("not", predicate);
    }
  }

  //TODO:brad: I think this should ideally be what createPredicate does

  /**
   * Create predicate from String. Arguments starting with capital
   * letter are instantiate as Variables.
   *
   * @param desc
   * @return
   */
  private static Predicate createPredVars(String desc) {
    desc = desc.trim();
    int opar = desc.indexOf('(');     // open parenthesis
    int cpar = desc.lastIndexOf(')'); // close parenthesis
    if (opar == -1 || cpar == -1) {
      return null;
    }
    String name = desc.substring(0, opar);
    String args = desc.substring(opar + 1, cpar);
    List<Symbol> arglist = new ArrayList<>();

    List<String> tokens = Util.tokenizeArgs(args);

    for (String token : tokens) {
      arglist.add(createPredicateArg(token));
    }
    return new Predicate(name, arglist);
  }

  private static Symbol createPredicateArg(String arg) {
    if (arg == null || arg.isEmpty()) {
      return null;
    }

    //Don't split on special characters or anything for escaped string, just use Symbol constructor on contents
    if (arg.charAt(0) == '\"' && arg.charAt(arg.length() - 1) == '\"') {
      return new Symbol(arg, "");
    }

    int opar = arg.indexOf('(');
    int cpar = arg.lastIndexOf(')');
    int ind = arg.indexOf(":");

    Symbol newArg;
    //TODO:brad: IDK if this is used anywhere, single quotes can be escaped, but they are not escape chars like " are, so this can get confusing, we should probably take it out. keeping it here for now, because I'm not sure where it's called.
    if (arg.startsWith("'") && arg.endsWith("'")) {
      log.warn("[createPredicate]single quotes case what is this for? " + arg);
      newArg = Factory.createSymbol(arg);
    } else if (opar > 0 && cpar == arg.length() - 1) {
      newArg = createPredicate(arg);
    } else if (ind >= 0) {
      String aname = arg.substring(0, ind);
      String atype = arg.substring(ind + 1);
      if (aname.startsWith("?") || aname.startsWith("!") || Character.isUpperCase(aname.charAt(0))) {
        newArg = new Variable(aname, atype);
      } else {
        newArg = new Symbol(aname, atype);
      }
    } else if (Character.isUpperCase(arg.charAt(0)) || arg.startsWith("?") || arg.startsWith("!")) {
      newArg = new Variable(arg, "");
    } else {
      newArg = Factory.createSymbol(arg);
    }
    return newArg;
  }
}
