/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

import static org.junit.Assert.*;

public class RefResolutionTest {
  private static final Logger log = LoggerFactory.getLogger(RefResolutionTest.class);

  // =========================================================================
  // EntityScore tests
  // =========================================================================

  @Test
  public void testEntityScoreConstructorAndAccessors() {
    EntityScore es = new EntityScore(true, 0.5, 2, 1.0);
    assertTrue(es.inMainClause());
    assertEquals(0.5, es.synProm(), 1e-9);
    assertEquals(2, es.recency());
    assertEquals(1.0, es.bonus(), 1e-9);
  }

  @Test
  public void testEntityScoreDefaults() {
    EntityScore es = new EntityScore(false, 0.0, 0, 0.0);
    assertFalse(es.inMainClause());
    assertEquals(0.0, es.synProm(), 1e-9);
    assertEquals(0, es.recency());
    assertEquals(0.0, es.bonus(), 1e-9);
  }

  // =========================================================================
  // Hypothesis tests
  // =========================================================================

  @Test
  public void testHypothesisConstructorAndAccessors() {
    Map<Variable, Symbol> assignments = new HashMap<>();
    Variable v = new Variable("X", "object");
    Symbol s = Factory.createSymbol("obj1");
    assignments.put(v, s);
    Hypothesis h = new Hypothesis(assignments, 0.9);
    assertEquals(0.9, h.likelihood(), 1e-9);
    assertEquals(1, h.assignments().size());
    assertEquals(s, h.assignments().get(v));
  }

  @Test
  public void testHypothesisMerge() {
    Variable v1 = new Variable("X", "object");
    Variable v2 = new Variable("Y", "object");
    Symbol s1 = Factory.createSymbol("obj1");
    Symbol s2 = Factory.createSymbol("obj2");

    Map<Variable, Symbol> a1 = new HashMap<>();
    a1.put(v1, s1);
    Hypothesis h1 = new Hypothesis(a1, 0.8);

    Map<Variable, Symbol> a2 = new HashMap<>();
    a2.put(v2, s2);
    Hypothesis h2 = new Hypothesis(a2, 0.5);

    Hypothesis merged = h1.merge(h2);
    assertEquals(0.8 * 0.5, merged.likelihood(), 1e-9);
    assertEquals(2, merged.assignments().size());
    assertEquals(s1, merged.assignments().get(v1));
    assertEquals(s2, merged.assignments().get(v2));
  }

  // =========================================================================
  // Property tests
  // =========================================================================

  @Test
  public void testPropertyPredicateForm() {
    Term t = Factory.createPredicate("red(X:object)");
    Property p = new Property(t);
    Term pred = p.predicateForm();
    assertEquals("dspred", pred.getName());
    assertEquals(3, pred.getArgs().size());
  }

  @Test
  public void testPropertyNegatedPredicateForm() {
    Term inner = Factory.createPredicate("red(X:object)");
    Term notTerm = new Term("not", inner);
    Property p = new Property(notTerm);
    Term pred = p.predicateForm();
    assertEquals("dspred", pred.getName());
    assertEquals("0.0", pred.get(1).getName());
    assertEquals("0.0", pred.get(2).getName());
  }

  @Test
  public void testPropertyNonDSPredicateForm() {
    Term t = Factory.createPredicate("blue(X:object)");
    Property p = new Property(t);
    Term nonDS = p.nonDSPredicateForm();
    assertEquals("blue", nonDS.getName());
  }

  // =========================================================================
  // RelevanceTheoreticCandidate tests
  // =========================================================================

  @Test
  public void testRelevanceTheoreticCandidate() {
    Symbol ref = Factory.createSymbol("objects_1:objects");
    RelevanceTheoreticCandidate rtc = new RelevanceTheoreticCandidate(ref, 0.75);
    assertEquals(ref, rtc.ref());
    assertEquals(0.75, rtc.relevance(), 1e-9);
  }

  // =========================================================================
  // RelevanceTheoreticCandidateWithProabability tests (preserve typo)
  // =========================================================================

  @Test
  public void testRelevanceTheoreticCandidateWithProabability() {
    Symbol ref = Factory.createSymbol("objects_2:objects");
    RelevanceTheoreticCandidate rtc = new RelevanceTheoreticCandidate(ref, 0.5);
    RelevanceTheoreticCandidateWithProabability rtcp = new RelevanceTheoreticCandidateWithProabability(rtc, 0.9);
    assertEquals(rtc, rtcp.candidate());
    assertEquals(0.9, rtcp.probability(), 1e-9);
  }

  // =========================================================================
  // Growler static methods tests
  // =========================================================================

  @Test
  public void testScaleRelevance() {
    // scaleRelevance(0) should be ~0 (logistic function centered at 0)
    double r = Growler.scaleRelevance(0.0);
    assertEquals(0.0, r, 1e-9);
  }

  @Test
  public void testWeightRelevance() {
    // All zeros -> should produce scaleRelevance(0)*4 = 0
    EntityScore allZero = new EntityScore(false, 0.0, 0, 0.0);
    double w = Growler.weightRelevance(allZero);
    assertEquals(0.0, w, 1e-9);
  }

  @Test
  public void testCrossMappingCandidateListMerge() {
    Symbol ref1 = Factory.createSymbol("obj1:objects");
    Symbol ref2 = Factory.createSymbol("obj2:objects");
    Variable v1 = new Variable("X", "objects");
    Variable v2 = new Variable("Y", "objects");

    RelevanceTheoreticCandidate rtc1 = new RelevanceTheoreticCandidate(ref1, 0.8);
    RelevanceTheoreticCandidate rtc2 = new RelevanceTheoreticCandidate(ref2, 0.6);

    SingleVarCandidateList svcl1 = new SingleVarCandidateList(v1, new LinkedList<>(),
        List.of(new RelevanceTheoreticCandidateWithProabability(rtc1, 0.9)));
    SingleVarCandidateList svcl2 = new SingleVarCandidateList(v2, new LinkedList<>(),
        List.of(new RelevanceTheoreticCandidateWithProabability(rtc2, 0.7)));

    List<CrossMappingCandidateList> list1 = Growler.generateCrossMappingCandidateList(svcl1);
    List<CrossMappingCandidateList> list2 = Growler.generateCrossMappingCandidateList(svcl2);

    assertEquals(1, list1.size());
    assertEquals(1, list2.size());

    List<CrossMappingCandidateList> merged = Growler.mergeCrossMappingCandidateLists(list1, list2);
    assertEquals(1, merged.size());
    assertEquals(0.9 * 0.7, merged.get(0).probability(), 1e-9);
    assertEquals(2, merged.get(0).bindings().size());
  }

  @Test
  public void testCmclToHypothesis() {
    Symbol ref = Factory.createSymbol("objects_1:objects");
    Variable v = new Variable("X", "objects");
    RelevanceTheoreticCandidate rtc = new RelevanceTheoreticCandidate(ref, 0.8);
    List<RelevanceTheoreticBinding> bindings = List.of(new RelevanceTheoreticBinding(v, rtc));
    CrossMappingCandidateList cmcl = new CrossMappingCandidateList(bindings, 0.75);

    Hypothesis h = Growler.cmclToHypothesis(cmcl);
    assertEquals(0.75, h.likelihood(), 1e-9);
    assertEquals(1, h.assignments().size());
    assertEquals(ref, h.assignments().get(v));
  }
}
