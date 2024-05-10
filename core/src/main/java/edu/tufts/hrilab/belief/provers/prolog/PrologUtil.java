/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.belief.provers.prolog;

import alice.tuprolog.Number;
import alice.tuprolog.Struct;
import alice.tuprolog.Var;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.text.CharacterIterator;
import java.text.StringCharacterIterator;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PrologUtil {

  private final static Logger log = LoggerFactory.getLogger(PrologUtil.class);
  private final static Pattern alphaNumericPattern = Pattern.compile("\\w+");
  private final static Pattern doublePattern = Pattern.compile("[\\x00-\\x20]*[-]?((\\p{Digit}+)(\\.)?((\\p{Digit}+)?))[\\x00-\\x20]*");

  public static String makePrologSafe(Term t) {
    //factors terms starting with 'not' into a prolog safe form that keeps the negation intact
    if (t.isNegated() && t.isTerm()) {
      Symbol negated = t.getArgs().get(0);
      List<Symbol> args = ((Term) negated).getArgs();
      String str = negated.getName();
      String camel = "not" + str.substring(0, 1).toUpperCase() + str.substring(1);
      t = new Term(camel,args);
    }

    StringBuilder safeName= new StringBuilder();
    safeName.append(t.getName()).append("(");

    String prefix="";
    for (Symbol sym : t.getArgs()) {
      if (sym.isTerm()) {
        safeName.append(prefix).append(makePrologSafe((Term) sym));
      } else if (sym.getName().startsWith("?")) {
        safeName.append(prefix).append(Factory.createSymbol(sym.getName().substring(1)));
      }else {
        String symName = sym.getName(); //Changing this from getName() to getString() so we don't strip type info. May cause bugs! 5/17/22, MF

        if (symName.startsWith("_") && isAlphaNumeric(symName.substring(1))) {
          // free variable with leading underscore
          safeName.append(prefix).append(Factory.createSymbol(symName));
        } else if (needsEscaping(symName)) {
          //Prolog docs say that escaped quotes are valid within strings, yet throws exceptions when doing so.
          //  Using decimal codes seems to get around this bug
          //Extend quotes to encapsulate all special characters to consider symbol a prolog string to prevent any prolog exceptions
          symName = symName.replace("\"","\\34\\");
          symName = symName.replace("'","\\39\\");
          symName = "\"" + symName + "\"";
          safeName.append(prefix).append(new Symbol(symName ));
        } else if (symName.startsWith("\"") && symName.endsWith("\"")) {
          //TODO: if this turns out to work, do in a better way
          //replace all internal ' " with code
          symName = symName.substring(1,symName.length()-1).replace("\"","\\34\\");
          symName = "\"" + symName.replace("'","\\39\\") + "\"";
          safeName.append(prefix).append(symName);
        }
        else {
          safeName.append(prefix).append(Factory.createSymbol(symName ));
        }
      }
      prefix=",";
    }
    safeName.append(")");

    //Issue where Term name itself contains special characters (e.g. in case of location name in semantics). Looks like
    //  strings of the form "\"string with special characters\"(whatever)" throws malformed prolog exception, while
    //  "\"string with special characters(whatever)\"" works correctly
    String safeNameString = safeName.toString();
    if (needsEscaping(t.getName()) || (t.getName().startsWith("\"") && t.getName().endsWith("\""))) {
      safeNameString = safeNameString.replace("\"","\\34\\");
      safeNameString = safeNameString.replace("'","\\39\\");
      safeNameString = "\"" + safeNameString + "\"";
    }

    return safeNameString;
  }

  public static Symbol undoMakePrologSafe(Symbol t) {
    t.setName(t.getName().replace("\\34\\","\""));
    t.setName(t.getName().replace("\\39\\","'"));
    return t;
  }

    /**
     * Helper method to check if string is alphanumeric (letter, digit, or underscore).
     *
     * @param str
     * @return
     */
  static private boolean isAlphaNumeric(String str) {
    Matcher matcher = alphaNumericPattern.matcher(str);
    return matcher.matches();
  }

  /**
   * Helper method used to determine if string needs to be escaped with''
   */
  static private boolean needsEscaping(String s) {
    if (s.startsWith("\"") && s.endsWith("\"")) {
      return false;
    }

    if (isDouble(s)) {
      return false;
    }

    return !isAlphaNumeric(s);
  }

  /**
   * Performs regex to check if string is a double (or float or int).
   *
   * This uses modified code originally from Java source code
   * found here: https://docs.oracle.com/javase/1.5.0/docs/api/java/lang/Double.html
   *
   * @param str
   * @return
   */
  static private boolean isDouble(String str) {
    Matcher matcher = doublePattern.matcher(str);
    return matcher.matches();
  }

  /**
   * Convert a prolog Term to a DIARC Term.
   *
   * @param term
   * @return
   */
  public static Symbol convertToAdeForm(alice.tuprolog.Term term) {
    return convertToAdeForm(term, new HashMap<>(), new HashMap<>());
  }


  private static String escapeSpecialCharacters(String desc) {
    if (desc == null || desc.isEmpty()) {
      return null;
    }
    Matcher matcher = alphaNumericPattern.matcher(desc);
    Matcher doubleMatcher = doublePattern.matcher(desc);
    if (!matcher.matches() && !doubleMatcher.matches()) {
      return "\"" + desc + "\"";
    }
    return desc;
  }
  /**
   * Convert a prolog Term to a DIARC Term, using map of nullary predicates to retain
   * nullary DIARC predicates since prolog can't handle nullary predicates.
   *
   * @param term prolog Term
   * @param nullaryPreds
   * @return
   */
  public static Symbol convertToAdeForm(alice.tuprolog.Term term, Map<String,String> nullaryPreds, Map<String,String> types) {
    Symbol output;
    if (term.isAtom()) {
      output = Factory.createSymbol(escapeSpecialCharacters(((Struct) term).getName()));
//      output = new Symbol(convertFunctorName(((Struct) term).getName()));
      if(nullaryPreds.containsKey(output.getName())) {
        String nullPred = nullaryPreds.get(output.getName());
        if(types.containsKey(output.getName())){
          nullPred = output.getName() + ":" + types.get(output.getName()) + "()";
          log.debug("nullary pred name: "+nullPred);
        }
        output = Factory.createPredicate(nullPred);
      }
      else if(types.containsKey(output.getName())){
        output = new Symbol(output.getName(),types.get(output.getName()));
      }
    } else if (term.isNumber()) {
      Number number = (Number) term;
      if (number.isInteger()) {
        output = new Symbol(Integer.toString(number.intValue()));
      } else {
        output = new Symbol(Double.toString(number.doubleValue()));
      }
    } else if (term.isCompound()) {
      Struct struct = (Struct) term;
      output = convertCompoundToAdeForm(struct, nullaryPreds, types);
    } else {
      output = null;
      log.warn(String.format("No conversion rule applicable to %s.", term));
    }

    log.debug("output = "+output);
    return output;
  }

  /**
   * Helper method to recursively construct DIARC Symbol/Predicate from compound prolog Term (i.e., struct).
   * @param s prolog struct
   * @param nullaryPreds
   * @return
   */
  public static Predicate convertCompoundToAdeForm(Struct s, Map<String,String> nullaryPreds, Map<String,String> types) {
    List<Symbol> args = new ArrayList<>();
    for (int i = 0; i < s.getArity(); i++) {
      if (s.getArg(i) instanceof Var) {
        //args.add(new edu.tufts.hrilab.fol.Symbol(((Var)s.getArg(i)).getName()));
        // Remove variable bindings (?) from string representation
        String name = (((Var) s.getArg(i)).toStringFlattened()).replaceAll("([A-Z])_[a-z]\\d+_[a-z]\\d+", "$1");
        //factor the type into account, convert to easier form
        if(types.containsKey(name)){
          args.add(new Symbol(name,types.get(name)));
        } else {
        args.add(new Symbol(name));
        }
      } else if (s.getArg(i) instanceof Struct) {
        Struct structTemp = (Struct) s.getArg(i);
        if (structTemp.isAtom()) {
          String name= structTemp.getName();
          //Mke sure this is actually okay - want to preserve quotes going through this pipeline
          String fullName= structTemp.toString();
          if (fullName.startsWith("\'") && fullName.endsWith("\'")) {
            name = "\"" + name + "\"";
          }
          if(nullaryPreds.containsKey(name)) {
            String nullPred = nullaryPreds.get(name);
            if(types.containsKey(name)){
              nullPred = name + ":" + types.get(name) + "()";
              log.debug("nullary pred name: "+nullPred);
            }
            args.add(Factory.createPredicate(nullPred));
          } else {
            //factor the type into account, convert to easier form
            if (types.containsKey(name)){
              args.add(new Symbol(name,types.get(name)));
          } else {
            args.add(new Symbol(name));
            }
          }
        } else if (structTemp.isCompound()) {
          args.add(convertCompoundToAdeForm(structTemp, nullaryPreds, types));
        }
      } else {
        args.add(new Symbol(s.getArg(i).toString()));
      }
    }

    //extract not-predicates
    if (s.getName().startsWith("not") && s.getName().substring(3,4).equals(s.getName().substring(3,4).toUpperCase())){
      String name = s.getName().substring(3, 4).toLowerCase() + s.getName().substring(4);
      Predicate negated = new Predicate(convertFunctorName(name), args);
      return new Predicate("not",negated);
    } else {
      return new Predicate(convertFunctorName(s.getName()), args);
    }
  }

  /**
   * Convert prolog functor name to DIARC functor name.
   *
   * @param functorName
   * @return
   */
  public static String convertFunctorName(String functorName) {
    switch (functorName) {
      case ":-":
        return"implies";
      case",":
        return  "and";
      case ";":
        return "or";
      default:
        return functorName;
    }
  }
}
