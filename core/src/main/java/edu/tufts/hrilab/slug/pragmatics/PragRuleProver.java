/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.pragmatics;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.common.UtteranceUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.fol.util.PragUtil;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static java.lang.Character.isUpperCase;

public class PragRuleProver {

  private Logger log = LoggerFactory.getLogger(this.getClass());
  protected Set<Symbol> leftSem;
  protected Set<Term> rightSem; // on record semantics  (direct)
  protected Set<Term> offRecordSem; // off record semantics (indirect)
  protected Set<Variable> vars;
  protected float score = -1.0f;
  protected float directness;
  private UtteranceType leftType;
  private UtteranceType rightType;
  private List<Symbol> listeners;
  private Symbol speaker;

  ///////////////////////////////////////
  public PragRuleProver() {
    leftSem = new HashSet<>();
    rightSem = new HashSet<>();
    offRecordSem = new HashSet<>();
    vars = new HashSet<>();
  }

  public PragRuleProver(String line) {
    leftSem = new HashSet<>();
    rightSem = new HashSet<>();
    offRecordSem = new HashSet<>();
    vars = new HashSet<>();
    this.parseRule(line);
  }

  ////////////////////////////////////////

  public List<Symbol> getListeners() {
    return listeners;
  }

  public Symbol getSpeaker() {
    return speaker;
  }

  public UtteranceType getLeftType() {
    return leftType;
  }

  public UtteranceType getRightType() {
    return rightType;
  }

  public Set<Symbol> getLeftHandSem() {
    return leftSem;
  }

  public void setLeftHandSem(Set<Symbol> t) {
    this.leftSem = t;
  }

  public Set<Term> getSemantics() {
    return rightSem;
  }

  public void setSemantics(Set<Term> semantics) {
    this.rightSem = semantics;
  }

  public Set<Term> getOffRecordSem() {
    return offRecordSem;
  }

  public void setOffRecordSem(Set<Term> semantics) {
    this.offRecordSem = semantics;
  }

  public float getScore() {
    return score;
  }

  public void setScore(float f) {
    this.score = f;
  }

  public float getDirectness() {
    return directness;
  }

  public void setDirectness(float f) {
    directness = f;
  }

  ////////////////////////////////////////
  public void parseRule(String line) {
    rightSem.clear();
    String[] splitRule = line.split(":=");

    String uttStr = splitRule[0];
    String semStr = splitRule[1];

    // handle utterance
    uttStr = uttStr.trim();
    if (uttStr.contains(";")) {
      String[] scoreSplit = uttStr.split(";");
      uttStr = scoreSplit[1].trim();
      this.score = Float.parseFloat(scoreSplit[0]);
    }

    Utterance uttForm = UtteranceUtil.createUtterance(uttStr);
    leftSem.add(uttForm.getSemantics());
    leftType = uttForm.getType();
    speaker = uttForm.getSpeaker();
    listeners = uttForm.getListeners();

    // handle utterance
    uttStr = uttStr.trim();
    if (uttStr.contains(";")) {
      String[] scoreSplit = uttStr.split(";");
      uttStr = scoreSplit[1].trim();
      this.score = Float.parseFloat(scoreSplit[0]);
    }

    // handle semantics
    String onRecStr;
    String offRecStr;

    if (semStr.contains(";")) {
      String[] semSplit = semStr.split(";");
      onRecStr = semSplit[0].trim();
      offRecStr = semSplit[1].trim();
    } else {
      onRecStr = semStr.trim();
      offRecStr = "";
    }

    //determine if these semantics are an utterance or not
    if (!onRecStr.isEmpty()) {
      String[] semArray = onRecStr.split("&");
      for (String s : semArray) {
        if (isUpperCase(s.charAt(0))) {
          Utterance rightUtt = UtteranceUtil.createUtterance(s);
          rightSem.add((Term) rightUtt.getSemantics());
          rightType = rightUtt.getType();
        } else {
          Term pred = Factory.createPredicate(s.trim());
          rightSem.add(pred);
          rightType = leftType;
        }
      }
    }

    if (!offRecStr.isEmpty()) {
      String[] semArray = offRecStr.split("&");
      for (String s : semArray) {
        Term pred = Factory.createPredicate(s.trim());
        offRecordSem.add(pred);
      }
    }

    //TODO:brad: where do these values come from?
    // default directness, determined by utterance type
    log.debug("PragRule.parseRule(): " + leftSem + " : " + leftType);
    if (score >= 0.0f) {
      directness = score;
    } else if (leftType.equals(UtteranceType.QUESTION)) {
      directness = 2.0f;
    } else if (leftType.equals(UtteranceType.INSTRUCT)) {
      directness = 3.0f;
    } else {
      directness = 1.0f;
    }
    log.debug("PragRule.directness " + directness);

  }

  @Override
  public String toString() {
    String retSet = "<" + score + ">;" + leftSem + " := ";
    for (Symbol t : rightSem) {
      retSet += t.toString() + " & ";
    }
    return retSet.substring(0, retSet.length() - 3);
  }

  ///////////////////////////////////////
  //public Map<Variable, Symbol> lhsApplicableMap(Utterance u) {
  //  return UtteranceUtil.getUtteranceBindings(uttForm, u);
  //}

//  public Map<Variable, Symbol> lhsApplicableMap(Map<Variable, Symbol> binding, Utterance u) {
//    return UtteranceUtil.getUtteranceBindings(binding, uttForm, u);
//  }

//  public ArrayList<Map<Variable, Symbol>> rhsApplicableMaps(Set<Term> terms) {
//    ArrayList<Map<Variable, Symbol>> retMaps = new ArrayList<Map<Variable, Symbol>>();
//    for (Term t2 : terms) {
//      for (Term t1 : offRecordSem) {
//        Map<Variable, Symbol> bindings = PragUtil.getTermBindings(t1, t2);
//        if (bindings != null) {
//          retMaps.add(bindings);
//        }
//      }
//    }
//    return retMaps;
//  }

  public ArrayList<Map<Variable, Symbol>> rhsApplicableMap(Term t) {
    ArrayList<Map<Variable, Symbol>> retMaps = new ArrayList<>();
    for (Symbol t1 : offRecordSem) {
      Map<Variable, Symbol> bindings = PragUtil.getSymbolBindings(t1, t);
      if (bindings != null) {
        retMaps.add(bindings);
      }
    }
    return retMaps;
  }


  public PragRuleProver getBoundVersion(Map<Variable, Symbol> binding) {

    if (binding == null) {
      return this;
    } else if (binding.isEmpty()) {
      return this;
    }

    PragRuleProver pr = new PragRuleProver();
    Set<Term> newTerms = new HashSet<>();
    Set<Term> newTermsOff = new HashSet<>();
    //Utterance u = UtteranceUtil.getBoundUtterance(binding, this.uttForm);

    for (Term t : this.rightSem) {
      newTerms.add(PragUtil.getBoundTerm(binding, t));
    }

    for (Term t : this.offRecordSem) {
      newTermsOff.add(PragUtil.getBoundTerm(binding, t));
    }

    //pr.setLeftHandSem(u);
    pr.setSemantics(newTerms);
    pr.setOffRecordSem(newTermsOff);
    pr.setDirectness(this.directness);

    pr.setScore(this.score);

    return pr;
  }

}
