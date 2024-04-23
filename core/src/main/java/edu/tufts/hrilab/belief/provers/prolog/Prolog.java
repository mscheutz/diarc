/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Attempt at a unified prover interface for belief.
 *
 * @author luca
 */
package edu.tufts.hrilab.belief.provers.prolog;

import alice.tuprolog.ExecutionContext;
import alice.tuprolog.InvalidTheoryException;
import alice.tuprolog.MalformedGoalException;
import alice.tuprolog.NoMoreSolutionException;
import alice.tuprolog.NoSolutionException;
import alice.tuprolog.SolveInfo;
import alice.tuprolog.Theory;
import alice.tuprolog.Var;
import alice.tuprolog.event.SpyListener;

import edu.tufts.hrilab.belief.provers.Prover;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.util.Utilities;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.io.InputStream;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Prolog implements Prover {
  private final static Logger log = LoggerFactory.getLogger(Prolog.class);

  private final alice.tuprolog.Prolog prolog = new alice.tuprolog.Prolog();
  private String thisStateName = null;

  private SpyListener spy;
  private Map<Term, Boolean> justification = new HashMap<>();
  private Map<String, String> nullaryPreds = new HashMap<>();
  private Map<String, String> types = new HashMap<>();

  public Prolog() {

    spy = spyEvent -> {
      alice.tuprolog.Engine e = spyEvent.getSnapshot();
      List<ExecutionContext> execStack = e.getExecutionStack();
      log.debug("execution: " + e);
      for (ExecutionContext c : execStack) {
        if (c.getCurrentGoal() != null && c.getDepth() < 2) {
          String goal = c.getCurrentGoal().toString();
          log.trace("goal: " + goal);
          if (thisStateName != null) {
            log.trace("statename: " + thisStateName);
            switch (thisStateName) {
              case "Eval":
                if (c.getHeadClause() != null) {// && !c.getHeadClause().getName().equals("not")) {
                  if (goal.equals("true")) {
                    Predicate p = PrologUtil.convertCompoundToAdeForm(c.getHeadClause(), nullaryPreds, types);
                    log.trace(p + " is true");
                    justification.put(p, true);
                  } else if (goal.equals("fail")) {
                    Predicate p = PrologUtil.convertCompoundToAdeForm(c.getHeadClause(), nullaryPreds, types);
                    log.trace(p + " is false");
                    justification.put(p, false);
                  }
                }
                break;
              case "Call":
                if (e.getNextStateName() == null && !goal.equals("true") && !goal.equals("fail")) {
                  Predicate p = PrologUtil.convertCompoundToAdeForm(c.getCurrentGoal(), nullaryPreds, types);
                  log.trace(p + " is true");
                  justification.put(p, true);
                }
                break;
              case "Back":
                if (e.getNextStateName() == null && !goal.equals("true") && !goal.equals("fail")) {
                  Predicate p = PrologUtil.convertCompoundToAdeForm(c.getCurrentGoal(), nullaryPreds, types);
                  log.trace(p + " is false");
                  justification.put(p, false);
                }
                break;
            }
          }
        }
      }
      thisStateName = e.getNextStateName();
    };

    prolog.addSpyListener(spy);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public Boolean querySupport(Term query) {
    // TODO: is this handling of "not" needed?
    if (query.getName().equals("not")) {
      List<Map<Variable, Symbol>> results = queryBelief(query.toUnnegatedForm());
      return results.isEmpty();
    } else {
      List<Map<Variable, Symbol>> results = queryBelief(query);
      return (!results.isEmpty());
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public Pair<Boolean, Map<Term, Boolean>> querySupportWithExplanation(Term query) {
    prolog.setSpy(true);
    justification.clear();
    boolean sln = querySupport(query);
    prolog.setSpy(false);
    log.debug("Spy justification: " + justification);

    // justification has been updated by spy
    // make copy before returning since justification is re-used by spy
    return Pair.of(sln, new HashMap<>(justification));
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public List<Map<Variable, Symbol>> queryBelief(Term query) {
    // to handle query variables that start with ! or ?
    query = edu.tufts.hrilab.fol.util.Utilities.actionToProlog(query);

    // sanitize and query prolog
    String plQuery = sanitize(query) + ".";
    List<Map<Variable, Symbol>> tmpResults = queryString(plQuery);

    // apply variable map to query results (if needed)
    List<Map<Variable, Symbol>> finalResults = new ArrayList<>();
    for (Map<Variable, Symbol> result : tmpResults) {
      Map<Variable, Symbol> tmpBinding = new HashMap<>();
      result.forEach((k, v) -> tmpBinding.put(Utilities.prologToAction(k), PrologUtil.undoMakePrologSafe(v)));
      finalResults.add(tmpBinding);
    }

    return finalResults;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public void assertBelief(Term belief) {
    //brad: previously this was asserta instead of assert, I don't think the change matters because this is never going to be a rule, but then again I'm not sure...
    if (!querySupport(belief)) {
      solve(sanitize(new Term("assert", belief)));
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public void assertBelief(String belief) {
    // TODO: EAK: converting to Predicate is completely unnecessary and is only being done to use the sanitize method.
    assertBelief(Factory.createPredicate(belief));
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public void retractBelief(Term belief) {
    log.debug("retracting belief " + belief);

    //retracts only facts and not rules
//    while (solve(sanitize(new Term("clause", belief, new Symbol("true")), true))) {
    solve(sanitize(new Term("retract", belief)));
//    }
    //solve(sanitize(new Term("retractall", belief)));
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void retractBelief(String belief) {
    // TODO: EAK: converting to Predicate is completely unnecessary and is only being done to use the sanitize method.
    retractBelief(Factory.createPredicate(belief));
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public void assertRule(Term head, List<Term> body) {
    StringBuilder rule = new StringBuilder(sanitize(head)).append(":-(");
    for (Term term : body) {
      rule.append(sanitize(term)).append(",");
    }
    rule.replace(rule.length() - 1, rule.length(), ")");

    assertRule(rule.toString());
  }


  /**
   * {@inheritDoc}
   */
  @Override
  public void assertRule(String rule) {
    try {
      prolog.solve("asserta((" + rule + ")).");
    } catch (Exception e) {
      log.error("Prolog assertRule error: " + rule, e);
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public void retractRule(Term head, List<Term> body) {
    StringJoiner rule = new StringJoiner(",", head.toString() + " :- ", "");
    for (Term bt : body) {
      rule.add(bt.toString());
    }

    retractRule(rule.toString());
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void retractRule(String rule) {
    try {
      log.trace("retracting: " + rule);
      prolog.solve("retractall(" + rule + ").");
    } catch (Exception e) {
      log.error("Prolog retractRule error: ", e);
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void initializeFromFiles(List<String> filenames) throws IOException {
    //clear any existing theory
    prolog.clearTheory();

    for (String f : filenames) {
      initializeFromFile(f);
    }
  }

  private void initializeFromFile(String filename) throws IOException {
    if (filename == null || filename.isEmpty()) {
      log.info("bad prolog filename provided not loading: " + filename);
      return;
    }

    InputStream in = getClass().getResourceAsStream(filename);

    if (in != null) {
      try {
        prolog.addTheory(new Theory(in));
      } catch (InvalidTheoryException e) {
        log.error("Invalid theory: " + filename, e);
      }
    } else {
      log.error("Configuration file not found: " + filename);
    }

  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public String getTheory() {
    return prolog.getTheory().toString();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public Object getInternalTheory() {
    return prolog.getTheory();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  synchronized public boolean setTheory(String theory) {
    try {
      prolog.setTheory(new Theory(theory));
      return true;
    } catch (InvalidTheoryException e) {
      log.error("Received invalid theory", e);
      return false;
    }
  }

  private List<Map<Variable, Symbol>> queryString(String query) {
    List<Map<Variable, Symbol>> results = new ArrayList<>();

    log.debug("querying Knowledge Base: " + query);
    try {
      SolveInfo sln = prolog.solve(query);
      while (sln.isSuccess()) {
        Map<Variable, Symbol> map = new HashMap<>();
        List<Var> bindings = sln.getBindingVars();
        for (Var v : bindings) {
          if (v.isBound()) {
            Symbol s = PrologUtil.convertToAdeForm(v.getTerm(), nullaryPreds, types);
            map.put(new Variable(v.getName()), s);
          }
        }
        if (results.isEmpty() || !map.isEmpty()) {
          results.add(map);
        }

        if (prolog.hasOpenAlternatives()) {
          sln = prolog.solveNext();
        } else {
          break;
        }
      }
    } catch (MalformedGoalException e) {
      log.error(String.format("Prolog Query Error during query '%s': ", query), e);
      results = null;
    } catch (NoSolutionException nmse) {
      log.error(String.format("No solution found for query %s", query));
    } catch (NoMoreSolutionException nmse) {
      log.error(String.format("No more solutions found for query %s", query));
    }

    log.debug("query results: " + results);
    return results;
  }

  @Override
  public String sanitize(Term term) {
    //save type information. TODO: figure out how to do this in a more belief specific way
    types.putAll(getTypeMap(term));

    //don't want type info
    String sanitized = PrologUtil.makePrologSafe(term);

    //regex to find all strings that precede "()" and don't contain "," or "("
    Matcher m = Pattern.compile("([^,\\(]*)(?=\\(\\))")
            .matcher(sanitized);
    while (m.find()) {
      String np = m.group();
      if (!np.isEmpty()) {
        nullaryPreds.put(np, np + "()");
      }
    }

    //brad: this way nullary predicates do not cause a syntax error
    sanitized = sanitized.replace("()", "");
    log.debug("sanitized term: " + sanitized);
    return sanitized;
  }

  /**
   * Generate semantic type map () for Term.
   * @param term
   * @return
   */
  private Map<String,String> getTypeMap(Term term) {
    Map<String,String> typeMap = new HashMap<>();

    // check term
    if (term.hasType()) {
      typeMap.put(term.getName(), term.getType());
    }

    // check term args
    for (Symbol arg : term.getArgs()) {
      if (arg.isTerm()) {
        typeMap.putAll(getTypeMap((Term)arg));
      } else if (arg.hasType()) {
        typeMap.put(arg.getName(), arg.getType());
      }
    }
    return typeMap;
  }

  private boolean solve(String statement) {
    try {
      String query = statement + ".";
      log.debug("to Solver: " + query);
      return prolog.solve(query).isSuccess();
    } catch (MalformedGoalException e) {
      log.error("Malformed prolog statement: " + statement, e);
      return false;
    }
  }

  private void assertArity(Term t) {
    String bareRule = t.getName() + "(";
    int numArgs = t.getArgs().size();
    for (int i = 0; i < numArgs; i++) {
      bareRule += "null,";
    }
    bareRule = bareRule.substring(0, bareRule.length() - 1);
    log.debug("asserting dynamic arity: " + bareRule);
    try {
      prolog.solve("asserta(" + bareRule + ")).");
    } catch (Exception e) {
      log.error(String.format("Belief Component Arity Assertion Error for '%s': ", bareRule), e);
    }
  }

}
