/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.util.PragUtil;

import java.io.Serial;
import java.io.Serializable;
import java.util.List;

public class Property implements Serializable {
  @Serial
  private static final long serialVersionUID = -9099527991981849806L;

  private final Term property;

  public Property(Term property) {
    this.property = property;
  }

  public boolean matches(Property prop) {
    Term thispred = this.predicateForm();
    Term thatpred = prop.predicateForm();
    Object x = thispred.get(0);
    Object y = thatpred.get(0);
    if (!(x instanceof Term) || !(y instanceof Term)) {
      return false;
    }
    if (PragUtil.getTermBindingsVars((Term) x, (Term) y) == null) {
      return false;
    }
    double xl = Double.parseDouble(thispred.get(1).toString());
    double yl = Double.parseDouble(thatpred.get(1).toString());
    return ((xl > 0.5) == (yl > 0.5)) || Math.abs(xl - yl) < 0.25;
  }

  public Property boundForm(List<Variable> t) {
    Term boundPred = new Term(property);
    boundPred = boundPred.copyWithNewVariableTypes(t);
    return new Property(boundPred);
  }

  public Term predicateForm() {
    if (!property.getName().equals("not")) {
      return new Term("dspred", property, Factory.createSymbol("1.0"), Factory.createSymbol("1.0"));
    } else {
      return new Term("dspred", property.get(0), Factory.createSymbol("0.0"), Factory.createSymbol("0.0"));
    }
  }

  public Term nonDSPredicateForm() {
    return property;
  }

  @Override
  public String toString() {
    return property.toString();
  }
}
