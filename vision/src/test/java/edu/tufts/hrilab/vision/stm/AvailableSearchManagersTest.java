/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

import org.junit.Test;

/**
 *
 * @author Evan Krause <evan.krause@tufts.edu>
 */
public class AvailableSearchManagersTest {
  
  @Test
  public void replaceNamedDescriptorsTest() {
    AvailableSearchManagers asm = new AvailableSearchManagers(null);
    Predicate definedTerm = Factory.createPredicate("handle(X)");
    List<Term> definition = new ArrayList<>();
    definition.add(Factory.createPredicate("partOf(orange,knife)"));
    definition = PredicateHelper.convertToVisionForm(definition);
    System.out.println("definition: " + definition);
    asm.nameDescriptors(definition, definedTerm);
    
    List<Term> descriptors = new ArrayList<>();
    descriptors.add(Factory.createPredicate("partOf(handle,knife)"));
    descriptors = PredicateHelper.convertToVisionForm(descriptors);
    System.out.println("descriptors: " + descriptors);
    
    List<Term> replacedDescriptors = asm.appendNamedDescriptors(descriptors);
    System.out.println("modified descriptors: " + replacedDescriptors);
    
    assertEquals("Did not get expected search terms.", "", replacedDescriptors);
  }
}
