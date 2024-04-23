/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief.provers.clingo;

import edu.tufts.hrilab.belief.provers.Prover;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.belief.converter.ConvertToClingo;
import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.rmi.RemoteException;
import java.util.*;
import java.util.stream.Collectors;


public class ClingoProver implements Prover {

  private Set<String> knowledgeBase;
  private ConvertToClingo toClingo;
  private ClingoWrapper clingoWrapper;
  private static Logger log = LoggerFactory.getLogger(ClingoProver.class);

  public ClingoProver() {
    this.knowledgeBase = new HashSet<>();
    this.toClingo = new ConvertToClingo();
    this.clingoWrapper = new ClingoWrapper();
  }

  @Override
  public Boolean querySupport(Term query) {
    // convert query to asp string
    // call clingo
    // if satisfiable return true
    // otherwise return false

    log.debug("called querySupport(Term query):" + query);
    List<String> bs = new LinkedList<>();
    Set<Variable> vars = new HashSet<>();
    bs.add(toClingo.convertTerm(query));
    vars.addAll(query.getVars());

    for (Variable v : vars) {
      bs.add(toClingo.getTypeOfVariable(v));
    }

    String constraint = ":- not " + StringUtils.join(bs, ',') + '.';
    log.debug(constraint);

    List<String> program = new LinkedList<>(this.knowledgeBase);
    program.add(constraint);
    log.debug("calling clingo wrapper");
    return clingoWrapper.isSatisfiable(StringUtils.join(program, '\n'));

  }

  @Override
  public Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query) {
    return Pair.of(false, new HashMap<>());
  }


  /**
   * Queries the Belief State about the term in the argument, and a set of
   * bindings corresponding to all of the found answers.
   *
   * @param query : a query in Term format
   * @return an ArrayList of bindings: here represented as a HashMap from
   * Variables to Symbols
   */
  @Override
  public List<Map<Variable, Symbol>> queryBelief(Term query) {
    // convert query to asp string
    // call clingo
    // if satisfaiable, parse the ans/n predicate
    // put arg1/val1, arg2/val2...argn/valn
    log.debug("called queryBelief(Term query):" + query);

    List<String> bs = new LinkedList<>();
    Set<Variable> vars = new HashSet<>();
    bs.add(toClingo.convertTerm(query));
    vars.addAll(query.getVars());
    for (Variable v : vars) {
      String type = toClingo.getTypeOfVariable(v);
      if (StringUtils.isNotEmpty(type))
        bs.add(type);
    }

    List<String> headArgs = new LinkedList<>();
    for (Variable v : query.getOrderedVars())
      headArgs.add(toClingo.convertVariable(v));

    String head = "ans" + (headArgs.isEmpty() ? "" :
            "(" + StringUtils.join(headArgs, ',') + ")");
    String body = ":-" + StringUtils.join(bs, ',') + '.';


    List<String> program = new LinkedList<>(this.knowledgeBase);
    program.add(head + body);

    program.add("#show ans/" + headArgs.size() + " .");

    log.trace(StringUtils.join(program, '\n'));

    return clingoWrapper.parseModels(StringUtils.join(program, '\n')
            , query.getOrderedVars(), "ans");
  }

  /**
   * Adds a single belief to the Belief State of the agent. Note:
   * new beliefs are added such that they take priority in unification to old
   * beliefs. queryBelief() will still return all asserted and inferred
   * bindings.
   *
   * @param belief
   */
  @Override
  public void assertBelief(Term belief) {
    // convert the term and add it to the knowledgebase
    this.knowledgeBase.add(toClingo.convertTerm(belief) + ".");
  }

  /**
   * Removes a single belief from the Belief State of the agent.
   *
   * @param belief
   */
  @Override
  public void retractBelief(Term belief) {
    Iterator<String> i = knowledgeBase.iterator();

    while (i.hasNext()) {
      String e = i.next();
      if (!e.contains(":-") && !e.contains("..")) {
        Term t = Factory.createPredicate(e.substring(0, e.length() - 1));
        log.trace("retract belief: "+t);
        if (t != null && t.instanceOf(belief)) {
          i.remove();
        }
      }
    }
  }

  /**
   * @param head
   * @param body
   */
  @Override
  public void assertRule(Term head, List<Term> body) {
    // create a rule string
    // then add it to the kb
    this.knowledgeBase.add(toClingo.convertRule(head, body));
  }

  public List<Pair<Term, List<Term>>> getRules() {
    List<String> rules = knowledgeBase.stream().filter(s -> s.contains(":-")).collect(Collectors.toList());
    List<Pair<Term,List<Term>>> parsedRules = new ArrayList<>();
    for (String rawRule : rules) {
      String[] rule = rawRule.split(":-");
      Term head = Factory.createPredicate(rule[0]);
      Predicate premises;
      if (rule[1].startsWith("(")) {
        premises = Factory.createPredicate("tmp" + rule[1]);
      } else {
        premises = Factory.createPredicate("tmp("+rule[1]+")");
      }
      List<Term> body = new ArrayList(premises.getArgs());
      Pair<Term, List<Term>> pair = Pair.of(head, body);
      parsedRules.add(pair);
    }

    return parsedRules;
  }


  @Override
  public void retractRule(Term head, List<Term> body) {
    // create a rule string
    // then remove it from the kb
    this.knowledgeBase.remove(toClingo.convertRule(head, body));
  }

  /**
   * initialize the knowledge base from a file
   */
  @Override
  public void initializeFromFiles(List<String> files) throws IOException {
    // read the lines
    // add it to KB
    this.knowledgeBase.clear();

    for (String f : files) {
      InputStream in = getClass().getResourceAsStream(f);

      String toAdd = "";
      BufferedReader br = new BufferedReader(new InputStreamReader(in));
      String line;
      while ((line = br.readLine()) != null) {
        if (line.startsWith("%*")) {
          // multi-line comment
          log.error("Cannot currently handle multi-line comments. File: " + f);
          return;
        } else if (line.startsWith("%")) {
          // single line comment -- ignore line
          continue;
        } else if (line.contains("%")) {
          // comment after valid code -- remove the comment part
          int commentIndex = line.indexOf("%");
          line = line.substring(0, commentIndex);
        }

        toAdd += line.trim();
        if (toAdd.endsWith(".")) {
          knowledgeBase.add(toAdd);
          toAdd = "";
        }
      }
    }
  }

  /**
   * Get the theory representation as a string in target language format.
   * Returned string depends on prover implementation.
   *
   * @return internal theory representation
   */
  @Override
  public String getTheory() {
    return StringUtils.join(this.knowledgeBase, "\n\n") + "\n\n";
  }

  /**
   * Get the theory representation in its source format
   *
   * @return internal theory representation
   */
  @Override
  public Object getInternalTheory() {
    // not supported
    log.error("ClingoProver prover doesn't support this functionality. " +
            "Ignoring object getInternalTheory() call.");
    return null;
  }

  /**
   * Set the theory (aka beliefs aka assumptions).
   *
   * @return true if theory was set successfully
   */
  @Override
  public boolean setTheory(String theory) {
    this.knowledgeBase.clear();
    this.knowledgeBase.addAll(Arrays.asList(theory.split("\n")));
    return true;
  }

  public List<Map<Variable, Symbol>> queryString(String query) {
    // not supported
    log.error("ClingoProver prover doesn't support this functionality. " +
            "Ignoring queryString(String query) call.");
    return null;
  }

  public Boolean queryBelief(String query) {
    // not supported
    log.error("ClingoProver prover doesn't support this functionality. " +
            "Ignoring queryBelief(String query) call.");
    return null;
  }

  @Override
  public void assertBelief(String belief) {
    this.knowledgeBase.add(belief+".");
  }

  public void retractBelief(String belief) {
    this.knowledgeBase.remove(belief);
  }

  public void assertRule(String belief) {
    this.knowledgeBase.add(belief+".");
  }

  public void retractRule(String belief) {
    this.knowledgeBase.remove(belief);
  }

  @Override
  public String sanitize(Term term) {
    return toClingo.convertTerm(term);
  }

}
