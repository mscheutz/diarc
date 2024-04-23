/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief.converter;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.StringUtils;

import java.util.HashSet;
import java.util.List;
import java.util.LinkedList;
import java.util.Set;

public class ConvertToClingo {

  public String convertRule(Term head, List<Term> body) {
    String a = head != null ? convertTerm(head) : "";
    List<String> bs = new LinkedList<>();
    Set<Variable> vars = new HashSet<>();
    for (Term b : body) {
      bs.add(this.convertTerm(b));
      vars.addAll(b.getVars());
    }


    for (Variable v : vars) {
      bs.add(this.getTypeOfVariable(v));
    }


    return a + ":-" + StringUtils.join(bs, ',') + '.';
  }

  public String convertTerm(Term term) {
    String name = term.getName();
    List<String> args = new LinkedList<>();
    for (Symbol arg : term.getArgs()) {
      args.add(this.convertSymbol(arg));
    }

    if (args.isEmpty()) {
      return name;
    } else {
      return term.isNegated() ? "not " : "" + name + "("
              + StringUtils.join(args, ",") + ")";
    }
  }

  public String convertVariable(Variable var) {
    // valid regex: _[A-Z][A-Z0-9a-z]*
    String name = var.getName();
    name = name.replaceAll("[^A-Za-z0-9]", "");
    if (!('A' <= name.charAt(0) && 'Z' >= name.charAt(0)) && name.charAt('0') != '_') {
      name = StringUtils.capitalize(name);
    }

    return name;
  }

  public String getTypeOfVariable(Variable var) {
    String type = var.getType();
    if (type == null || type.isEmpty()) {
      return null;
    }

    return type + "(" + this.convertVariable(var) + ")";
  }

  public String convertSymbol(Symbol symbol) {
    if (symbol.isTerm())
      return this.convertTerm((Term) symbol);
    if (symbol.isVariable())
      return this.convertVariable((Variable) symbol);

    /*** it's a constant otherwise */
    String name = symbol.getName();

    /*ASP constants start with lowercase or can be numbers OR can be in quotes*/
    if (name.matches("^_*[a-z0-9][A-Za-z0-9]*$") || name.matches("^\".*\"$")) {
      return name;
    } else {
      return name.toLowerCase();
    }
  }
}
