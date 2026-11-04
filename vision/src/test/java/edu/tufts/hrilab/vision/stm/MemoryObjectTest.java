/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import static org.junit.Assert.*;

import edu.tufts.hrilab.vision.util.PredicateHelper;
import org.junit.Before;
import org.junit.Test;

/**
 *
 * @author Evan Krause <evan.krause@tufts.edu>
 */
public class MemoryObjectTest {

  private MemoryObject mo;

  public MemoryObjectTest() {

  }

  @Before
  public void instantiateTestMemoryObject() {
    // instantiate Test MemoryObject
    mo = new MemoryObject();
    mo.setTokenId(0);
    mo.setVariable(Factory.createVariable("X", "object"));
    mo.setDetectionConfidence(0.5f);
    mo.addDescriptor(PredicateHelper.createPredicate("knife(X)"), 1.0f);

    // add handle of object
    MemoryObject mo_handle = new MemoryObject();
    mo_handle.setTokenId(1);
    mo.setVariable(Factory.createVariable("Y", "object"));
    mo_handle.setDetectionConfidence(0.6f);
    mo_handle.addDescriptor(PredicateHelper.createPredicate("handle(Y)"), 1.0f);
    mo.addChild(mo_handle);

    // add handle relation to knife
    Term relation_term = PredicateHelper.createPredicate("on(Y,X)");
    mo.addRelation(0.85f, relation_term, mo_handle, true);

    // add blade of object
    MemoryObject mo_blade = new MemoryObject();
    mo_blade.setTokenId(2);
    mo.setVariable(Factory.createVariable("Z", "object"));
    mo_blade.setDetectionConfidence(0.7f);
    mo_blade.addDescriptor(PredicateHelper.createPredicate("blade(Z)"), 1.0f);
    mo.addChild(mo_blade);

    // add blade relation to knife
    relation_term = PredicateHelper.createPredicate("on(Z,X)");
    mo.addRelation(0.8f, relation_term, mo_blade, true);

    // add grasp point
    MemoryObject mo_grasp_point = new MemoryObject();
    mo_grasp_point.setTokenId(3);
    mo.setVariable(Factory.createVariable("W", "object"));
    mo_grasp_point.setDetectionConfidence(0.9f);
    mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 1.0f);
    mo.addChild(mo_grasp_point);

    // add grasp point relations
    relation_term = PredicateHelper.createPredicate("near(W,Z)");
    mo_blade.addRelation(0.85f, relation_term, mo_grasp_point, true);

    // add grasp point
    mo_grasp_point = new MemoryObject();
    mo_grasp_point.setTokenId(4);
    mo.setVariable(Factory.createVariable("W", "object"));
    mo_grasp_point.setDetectionConfidence(0.95f);
    mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 1.0f);
    mo.addChild(mo_grasp_point);

    // add grasp point relations
    relation_term = PredicateHelper.createPredicate("near(W,Y)");
    mo_handle.addRelation(0.8f, relation_term, mo_grasp_point, true);

    // add grasp point
    mo_grasp_point = new MemoryObject();
    mo_grasp_point.setTokenId(5);
    mo.setVariable(Factory.createVariable("W", "object"));
    mo_grasp_point.setDetectionConfidence(1.0f);
    mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 1.0f);
    mo.addChild(mo_grasp_point);

    // add grasp point relations
    relation_term = PredicateHelper.createPredicate("near(W,Y)");
    mo_handle.addRelation(0.75f, relation_term, mo_grasp_point, true);
  }

  @Test
  public void testGetConfidence() {
    List<Float> confs;
    List<Term> constraints = new ArrayList<>();
    Map<Variable, Symbol> bindings = new HashMap<>();
    Variable varX = new Variable("X");
    Variable varY = new Variable("Y");
    Variable varZ = new Variable("Z");

    ////////////////
//    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
//    confs = MemoryObjectUtil.getConfidence(mo, constraints, bindings);
//
//    assertEquals("Did not find correct number of bindings.", 1, bindings.size());
//    assertEquals("Did not find correct grasp_point(X).", "object5", bindings.get(varX).getName());
//    assertEquals("Did not find confidence for each constraint.", confs.size(), constraints.size());
//    assertEquals("Did not find correct confidence value.", 1.0, (double) confs.get(0), 0.0001);
//
//    ////////////////
//    constraints.clear();
//    bindings.clear();
//    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
//    constraints.add(PredicateHelper.createPredicate("near(X,Z)"));
//    constraints.add(PredicateHelper.createPredicate("blade(Z)"));
//    confs = MemoryObjectUtil.getConfidence(mo, constraints, bindings);
//
//    assertEquals("Did not find correct number of bindings.", 2, bindings.size());
//    assertEquals("Did not find correct grasp_point(X).", "object3", bindings.get(varX).getName());
//    assertEquals("Did not find correct blade(Z).", "object2", bindings.get(varZ).getName());
//    assertEquals("Did not find confidence for each constraint.", confs.size(), constraints.size());
//    assertEquals("Did not find correct confidence value.", 0.9, confs.get(0), 0.0001);
//    assertEquals("Did not find correct confidence value.", 0.85, confs.get(1), 0.0001);
//    assertEquals("Did not find correct confidence value.", 0.7, confs.get(2), 0.0001);
    //////////////// Disjoint constraints
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("blade(Z)"));
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("near(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("handle(Y)"));
    confs = MemoryObjectUtil.getConfidence(mo, constraints, bindings);

    assertEquals("Did not find correct number of bindings.", 3, bindings.size());
    assertEquals("Did not find correct number of confidence values.", 4, confs.size());

    //////////////// over constrainted - not satisfiable
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("handle(Y)"));
    constraints.add(PredicateHelper.createPredicate("blade(Z)"));
    constraints.add(PredicateHelper.createPredicate("near(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("near(X,Y)"));
    confs = MemoryObjectUtil.getConfidence(mo, constraints, bindings);

    assertEquals("Did not find correct number of bindings.", 0, bindings.size());
    assertEquals("Did not find correct number of confidence values.", 0, confs.size());

    ////////////////
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("on(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("knife(Z)"));
    confs = MemoryObjectUtil.getConfidence(mo, constraints, bindings);

    assertEquals("Did not find correct number of bindings.", 0, bindings.size());
    assertEquals("Did not find confidence for each constraint.", 0, confs.size());

    /////////////////////////////////
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("near(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("handle(Z)"));
    constraints.add(PredicateHelper.createPredicate("on(Z,Y)"));
    constraints.add(PredicateHelper.createPredicate("knife(Y)"));
    confs = MemoryObjectUtil.getConfidence(mo, constraints, bindings);

    assertEquals("Did not find correct number of bindings.", 3, bindings.size());
    assertEquals("Did not find correct grasp_point(X).", "object4", bindings.get(varX).getName());
    assertEquals("Did not find correct knife(Y).", "object0", bindings.get(varY).getName());
    assertEquals("Did not find correct handle(Z).", "object1", bindings.get(varZ).getName());
    assertEquals("Did not find confidence for each constraint.", confs.size(), constraints.size());
    assertEquals("Did not find correct confidence value.", 0.95, confs.get(0), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.8, confs.get(1), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.6, confs.get(2), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.85, confs.get(3), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.5, confs.get(4), 0.0001);

    ///////////////////////////////// partially bound query
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("near(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("handle(Z)"));
    constraints.add(PredicateHelper.createPredicate("on(Z,object0)"));
    constraints.add(PredicateHelper.createPredicate("knife(object0)"));
    confs = MemoryObjectUtil.getConfidence(mo, constraints, bindings);

    assertEquals("Did not find correct number of bindings.", 2, bindings.size());
    assertEquals("Did not find correct grasp_point(X).", "object4", bindings.get(varX).getName());
    assertEquals("Did not find correct handle(Z).", "object1", bindings.get(varZ).getName());
    assertEquals("Did not find confidence for each constraint.", confs.size(), constraints.size());
    assertEquals("Did not find correct confidence value.", 0.95, confs.get(0), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.8, confs.get(1), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.6, confs.get(2), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.85, confs.get(3), 0.0001);
    assertEquals("Did not find correct confidence value.", 0.5, confs.get(4), 0.0001);
  }

  @Test
  public void testGetMemoryObjectBindings() {

    List<Term> constraints = new ArrayList<>();
    List<Map<Variable, MemoryObject>> bindings = new ArrayList<>();
    boolean allBindingsFound;
    Variable varX = new Variable("X");
    Variable varY = new Variable("Y");
    Variable varZ = new Variable("Z");

    /////////////////////////////////
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    allBindingsFound = MemoryObjectUtil.getMemoryObjectBindings(mo, constraints, bindings);

    assertTrue("Did not find bindings for all constraints.", allBindingsFound);
    assertEquals("Did not find all bindings for grasp_point(X).", 3, bindings.size());
    for (Map<Variable, MemoryObject> bindingsInstance : bindings) {
      assertEquals("Did not find correct number of bindings.", 1, bindingsInstance.size());
      assertNotNull("Did not find all bindings for grasp_point(X).", bindingsInstance.get(varX));
    }

    /////////////////////////////////
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("on(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("knife(Z)"));
    allBindingsFound = MemoryObjectUtil.getMemoryObjectBindings(mo, constraints, bindings);

    assertFalse("Should not have found bindings.", allBindingsFound);
    assertTrue("Should not have found bindings.", bindings.isEmpty());

    /////////////////////////////////
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("near(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("blade(Z)"));
    allBindingsFound = MemoryObjectUtil.getMemoryObjectBindings(mo, constraints, bindings);

    assertTrue("Did not find bindings for all constraints.", allBindingsFound);
    assertEquals("Did not find correct number of bindings options.", 1, bindings.size());
    assertNotNull("Did not find binding for blade(Z).", bindings.get(0).get(varZ));
    assertNotNull("Did not find binding for grasp_point(X).", bindings.get(0).get(varX));

    /////////////////////////////////
    constraints.clear();
    bindings.clear();
    constraints.add(PredicateHelper.createPredicate("grasp_point(X)"));
    constraints.add(PredicateHelper.createPredicate("near(X,Z)"));
    constraints.add(PredicateHelper.createPredicate("handle(Z)"));
    constraints.add(PredicateHelper.createPredicate("on(Z,Y)"));
    constraints.add(PredicateHelper.createPredicate("knife(Y)"));
    allBindingsFound = MemoryObjectUtil.getMemoryObjectBindings(mo, constraints, bindings);

    assertTrue("Did not find bindings for all constraints.", allBindingsFound);
    assertEquals("Did not find correct number of bindings options.", 2, bindings.size());
    for (Map<Variable, MemoryObject> bindingsInstance : bindings) {
      assertEquals("Did not find correct number of bindings.", 3, bindingsInstance.size());
      assertNotNull("Did not find all bindings for grasp_point(X).", bindingsInstance.get(varX));
      assertNotNull("Did not find all bindings for knife(Y).", bindingsInstance.get(varY));
      assertNotNull("Did not find all bindings for handle(Z).", bindingsInstance.get(varZ));
    }

  }
}
