/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.junit.BeforeClass;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.*;

import static org.junit.Assert.assertTrue;

public class BeliefComponentTest {
  private static BeliefComponent component;
  private static Logger log = LoggerFactory.getLogger(BeliefComponentTest.class);

  public BeliefComponentTest() {
  }

  @BeforeClass
  public static void init() {
    component = DiarcComponent.createInstance(BeliefComponent.class, "-initfile demos.pl -initfile agents/agents.pl");
  }

  @Test
  public void testRetract() {
    // String[] words = utterance.split(" ");
    // ArrayList<String> complete = new ArrayList<>(Arrays.asList(words));
    log.info("Testing utterance");
    //log.info(component.getAllBeliefs());
    Map<Variable,Symbol> outcome = new HashMap<>();
    Map<Variable,Symbol> desired = new HashMap<>();
    desired.put(new Variable("X"),new Symbol("a"));
    desired.put(new Variable("X"),new Symbol("b"));
    desired.put(new Variable("X"),new Symbol("c"));

    try {
      // component.addWords(complete);

      Term t = new Term("clear","a");
      component.assertBelief(t);
      t = new Term("clear","b");
      component.assertBelief(t);
      t = new Term("obtuse","c");
      component.assertBelief(t);
      List<Term> s = new ArrayList<>();
      t = new Term("obtuse","X");
      s.add(t);
      t = new Term("clear","X");
      component.assertRule(t,s);
      List<Map<Variable,Symbol>> bindings = component.queryBelief(t);

      log.info("outcome: " + bindings);
      outcome = bindings.get(0);
      assertTrue(outcome.equals(desired));

      t = new Term("clear","X");
      component.retractBelief(t);

      desired = new HashMap<>();
      desired.put(new Variable("X"),new Symbol("c"));
      bindings = component.queryBelief(t);

      log.info("outcome: " + bindings);
      outcome = bindings.get(0);

    } catch (Exception e) {
      log.error("Error calling test utterance", e);
    }
    assertTrue(outcome.equals(desired));
  }

}
