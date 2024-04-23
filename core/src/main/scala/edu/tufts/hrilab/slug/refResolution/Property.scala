/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution

import edu.tufts.hrilab.fol.util.PragUtil
import edu.tufts.hrilab.fol.{Factory, Term, Variable}

import scala.collection.JavaConverters._

@SerialVersionUID(-9099527991981849806L)
class Property(val property:Term) extends Serializable{

  def matches(prop:Property):Boolean = {

    //Brad: in the case of predicate properties, .predicateForm creates DSPreds with 1.0 bel and 1.0 plausibility
    val thispred = this.predicateForm
    val thatpred = prop.predicateForm
    (thispred.get(0),thatpred.get(0)) match{
      case (x: Term, y: Term) =>
        if(Option(PragUtil.getTermBindingsVars(x, y)).isEmpty) false
        else{
          val xl = thispred.get(1).toString.toDouble
          val yl = thatpred.get(1).toString.toDouble
          ((xl > 0.5) == (yl > 0.5)) || Math.abs(xl - yl) < 0.25
        }
      case other => false
    }
  }

  def boundForm(t: Seq[Variable]): Property = {
    var boundPred = new Term(property)
    boundPred = boundPred.copyWithNewVariableTypes(t.asJava)
    new Property(boundPred)
  }
  def predicateForm: Term = {
    if (!property.getName.equals("not")) {
      new Term("dspred", property, Factory.createSymbol("1.0"), Factory.createSymbol("1.0"))
    } else {
      new Term("dspred", property.get(0), Factory.createSymbol("0.0"), Factory.createSymbol("0.0"))
    }
  }
  def nonDSPredicateForm: Term = property

  override def toString: String =  property.toString
}
