/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pddl;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.StringJoiner;

public class Generator {
  private static Logger log = LoggerFactory.getLogger(Generator.class);

  public static String generate(Symbol symbol) {
    if (symbol.isTerm()) {
      return generateFromTerm((Term) symbol);
    } else {
      return generateFromSymbol(symbol);
    }
  }

  public static String generateTyped(Symbol symbol) {
    if (symbol.isTerm()) {
      return generateTypedFromTerm((Term) symbol);
    } else {
      return generateTypedFromSymbol(symbol);
    }
  }

  private static String generateFromTerm(Term term) {
    // handle negated predicates
    boolean isNegated = false;
    if (term.isNegated()) {
      isNegated = true;
      term = term.toUnnegatedForm();
    }

    // build pddl String
    StringJoiner joiner;
    if (isNegated) {
      joiner = new StringJoiner(" ", "(not(", "))");
    } else {
      joiner = new StringJoiner(" ", "(", ")");
    }

    // handle predicate name/operator
    if (Operators.DIARC_TO_PDDL.containsKey(term.getName())) {
      joiner.add(Operators.DIARC_TO_PDDL.get(term.getName()));
    } else {
      joiner.add(term.getName());
    }

    // handle args
    for (Symbol arg : term.getArgs()) {
      if (arg.isTerm()) {
        joiner.add(generateFromTerm((Term)arg));
      } else {
        joiner.add(generateFromSymbol(arg));
      }
    }
    return joiner.toString();
  }

  private static String generateTypedFromTerm(Term term) {
    // TODO: does this need to handle Operators.OPERATORS and negations??

    StringJoiner joiner = new StringJoiner(" ", "(", ")");
    joiner.add(term.getName());
    for (Symbol symbol : term.getArgs()) {
      String generatedType = generateTyped(symbol);
      if(generatedType.equals("")) {
        return null;
      }
      joiner.add(generatedType);
    }
    return joiner.toString();
  }

  private static String generateFromSymbol(Symbol symbol) {
    return symbol.getName();
  }

  private static String generateTypedFromSymbol(Symbol symbol) {
    if (symbol.hasType()) {
      return symbol.getName() + " - " + symbol.getType();
    }

    log.warn("Symbol does not contain type information: " + symbol);
    return symbol.getName();
  }

}
