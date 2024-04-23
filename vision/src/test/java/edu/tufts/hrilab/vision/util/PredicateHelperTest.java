/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.util;

import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.util.Utilities;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class PredicateHelperTest {

  @Test
  public void testConvertToVisionForm() {

    List<Term> descriptors = new ArrayList<>();
    descriptors.add(PredicateHelper.createPredicate("on(grasp_point, VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("on(VAR1, VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("object(VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("grasp_point(VAR1)"));
    descriptors.add(PredicateHelper.createPredicate("plate(VAR0)"));

    List<Term> convertedDescriptors = PredicateHelper.convertToVisionForm(descriptors);

    List<Term> desiredDescriptors = new ArrayList<>();
    desiredDescriptors.add(PredicateHelper.createPredicate("on(VAR1, VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("grasp_point(VAR1)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("on(VAR1, VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("grasp_point(VAR1)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("object(VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("plate(VAR0)"));

//    assertTrue("convertedDescriptors: " + convertedDescriptors, PredicateHelper.predicatesMatch(convertedDescriptors, desiredDescriptors));

    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    descriptors.clear();
    descriptors.add(PredicateHelper.createPredicate("on(grasp_point(ACTION_VAR0), VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("on(ACTION_VAR0, VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("object(VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("grasp_point(ACTION_VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("plate(VAR0)"));

    convertedDescriptors = PredicateHelper.convertToVisionForm(descriptors);

    desiredDescriptors.clear();
    desiredDescriptors.add(PredicateHelper.createPredicate("on(VAR1, VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("grasp_point(NEWVAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("object(VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("plate(VAR0)"));

    assertTrue("convertedDescriptors: " + convertedDescriptors, Utilities.predicatesMatch(convertedDescriptors, desiredDescriptors));

    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    descriptors.clear();
    descriptors.add(PredicateHelper.createPredicate("on(handle(NEWVAR1), VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("on(VAR1, VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("object(VAR0)"));
    descriptors.add(PredicateHelper.createPredicate("grasp_point(VAR1)"));
    descriptors.add(PredicateHelper.createPredicate("plate(VAR0)"));

    convertedDescriptors = PredicateHelper.convertToVisionForm(descriptors);

    desiredDescriptors.clear();
    desiredDescriptors.add(PredicateHelper.createPredicate("on(NEWVAR1, VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("handle(NEWVAR1)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("on(VAR1, VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("grasp_point(NEWVAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("object(VAR0)"));
    desiredDescriptors.add(PredicateHelper.createPredicate("plate(VAR0)"));

    assertTrue("convertedDescriptors: " + convertedDescriptors, Utilities.predicatesMatch(convertedDescriptors, desiredDescriptors));

  }
}
