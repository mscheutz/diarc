/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.*;

import edu.tufts.hrilab.vision.stm.MemoryObjectUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

public class VisionComponentTest {
  private static VisionComponent component;
  private static Logger log = LoggerFactory.getLogger(VisionComponentTest.class);

  public VisionComponentTest() {
  }

  @BeforeClass
  public static void init() {
    component = DiarcComponent.createInstance(VisionComponent.class, "-hideControls");
  }

  @Test
  public void faceDetectionTest() {
    // setup capture with image
    component.reinitializeVisionComponent(0, 0);

    // create search predicates and run tests
    Predicate descriptor = Factory.createPredicate("face(X)");
    runSearch(descriptor, 1);
  }

  @Test
  public void anotherFaceDetectionTest() {
    // setup capture with image
    component.reinitializeVisionComponent(0, 0);

    // create search predicates and run tests
    Predicate descriptor = Factory.createPredicate("face(X)");
    runSearch(descriptor, 1);
  }

  //TODO: capture better/cleaner pcd file to test with
  @Ignore
  @Test
  public void clusterDetectionTest() {
    // setup capture with image
    component.reinitializeVisionComponent(-Math.PI / 4.0, 1.5);

    // create search predicates and run tests
    Predicate descriptor = Factory.createPredicate("object(X)");
    runSearch(descriptor, 1);
  }

  @Test
  public void clusterGraspDetectionTest() {
    // setup capture with image
    component.reinitializeVisionComponent(-Math.PI / 4.0, 1.5);

    // create search predicates and run tests
    List<Predicate> descriptors = new ArrayList();
    descriptors.add(Factory.createPredicate("cluster(Y)"));
    descriptors.add(Factory.createPredicate("grasp_point(X)"));
    descriptors.add(Factory.createPredicate("on(X,Y)"));
    runSearch(descriptors, 1);
  }

  @Test
  public void learnMedkitTest() {
    // currently doesn't matter what the image is, just that it's a pcd file
    component.reinitializeVisionComponent(-Math.PI / 4.0, 1.5);

    // first make sure vision doesn't know what a medkit is
    Predicate medkit = Factory.createPredicate("medkit(X)");
    long medkitTypeId = -2L;
    medkitTypeId = component.getTypeId(medkit);
    assertEquals("How does vision already know what a medkit is?", -1L, medkitTypeId);

    // next instantiate search based on a description
    List<Predicate> description = new ArrayList<>();
    description.add(Factory.createPredicate("white(X)"));
    description.add(Factory.createPredicate("box(X)"));
    description.add(Factory.createPredicate("red(Y)"));
    description.add(Factory.createPredicate("cross(Y)"));
    description.add(Factory.createPredicate("on(Y,X,on)"));
    long descriptionTypeId = -2L;
    descriptionTypeId = component.getTypeId(description);
    assertTrue("Could not instantiate search for \"white box with a red cross on it.\"", -1L != descriptionTypeId);

    // name the search
    boolean namedSearch = false;
    namedSearch = component.nameDescriptors(descriptionTypeId, medkit);
    assertTrue("Could not name the search/description.", namedSearch);

    // make sure that we can search based on "medkit" now
    medkitTypeId = component.getTypeId(medkit);
    assertTrue("Vision still doesn't know what a medkit is!", -1L != medkitTypeId && medkitTypeId == descriptionTypeId);

    //TODO: actually look for medkit to make sure visual search works

    // tear down search
    component.stopAndRemoveType(medkitTypeId);
  }

  private void runSearch(Predicate descriptor, int numExpectedResults) {
    ArrayList<Predicate> descriptors = new ArrayList<>();
    descriptors.add(descriptor);
    runSearch(descriptors, numExpectedResults);
  }

  private void runSearch(List<Predicate> descriptors, int numExpectedResults) {
    // create new search
    Long typeId = -1L;
    typeId = component.getTypeId(descriptors);
    assertTrue("Invalid typeId returned.", typeId != -1L);

    // get search results
    List<MemoryObject> memoryObjects = null;
    memoryObjects = component.getTokens(typeId, 0.5);
    assertNotNull("Search results null.", memoryObjects);
    assertTrue(new StringBuilder("Incorrect number of search results found. Expected ").
                    append(numExpectedResults).append(" but found ").
                    append(memoryObjects.size()).append(" results.").toString(),
            memoryObjects.size() == numExpectedResults);

    // tear down search
    component.stopAndRemoveType(typeId);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////// these need to be turned into proper junit tests /////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  private void testLearn() {
//    Term descriptors = Factory.createPredicate("on(grasp_point,partOf(handle,knife))");
    Term descriptors = Factory.createPredicate("ball(X)");
//    Term learningTerm = Factory.createPredicate("looksLike(partOf(orange,knife),handle)");
    Term learningTerm = Factory.createPredicate("instanceOf(object,ball)");

    // start vision search
    log.info("[sandbox] starting visual search...");
    Long typeId = component.getTypeId(descriptors);

    if (typeId == -1L) {
      log.info("Cannot instantiate search: " + descriptors);

      // get thing vision doesn't know about
      List<Symbol> unsatisfiableConstraints = component.getUnsatisfiableConstraints(descriptors);
      log.info("unsatisfiableConstraints: " + unsatisfiableConstraints);

      // learn thing vision doesn't know about
      List<Map<Variable, Symbol>> bindings = new ArrayList<>();
      bindings.add(new HashMap<>());
      component.learn(learningTerm, bindings);

      // now see if search is valid
      typeId = component.getTypeId(descriptors);

      if (typeId == -1L) {
        log.info("Still can't instantiate search after learning.");
        return;
      } else {
        List<Term> adjustedDescriptors = component.getDescriptors(typeId);
        log.info("Visual search adjustedDescriptors: " + adjustedDescriptors);
      }
    } else {
      log.info("Search instantiated: " + descriptors);
    }

    MemoryObject token = null;
    double conf = 0.5;
    List<Long> tokenIds = component.getTokenIds(typeId, conf);
//    if (tokenIds != null && !tokenIds.isEmpty()) {
//      Long tokenId = tokenIds.get(0);
//      token = getToken(tokenId, conf);
//      token.printSceneGraph();
//    }

    component.stopType(typeId);
  }

  private void testVisualSearch() {

    // start vision search
    log.info("[sandbox] starting visual search...");
    List<Predicate> descriptors = new ArrayList();
    int searchNum = 9;
    if (searchNum == 0) {
      descriptors.add(Factory.createPredicate("object(Y)"));
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("on(X,Y)"));
    } else if (searchNum == 1) {
      descriptors.add(Factory.createPredicate("object(Y)"));
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("top(Z,Y)"));
      descriptors.add(Factory.createPredicate("on(X,Z)"));
    } else if (searchNum == 2) {
      descriptors.add(Factory.createPredicate("mug(Y)")); //knife
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("on(X,Y)"));
    } else if (searchNum == 3) {
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("mug(Y)")); //knife
      descriptors.add(Factory.createPredicate("blade(W)"));
      descriptors.add(Factory.createPredicate("near(X,W)"));
      descriptors.add(Factory.createPredicate("on(W,Y)"));
    } else if (searchNum == 4) {
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("mug(Y)")); //knife
      descriptors.add(Factory.createPredicate("handle(Z)"));
      descriptors.add(Factory.createPredicate("blade(W)"));
      descriptors.add(Factory.createPredicate("near(X,Z)"));
      descriptors.add(Factory.createPredicate("near(X,W)"));
      descriptors.add(Factory.createPredicate("on(Z,Y)"));
      descriptors.add(Factory.createPredicate("on(W,Y)"));
    } else if (searchNum == 5) {
      descriptors.add(Factory.createPredicate("knife(Y)")); //knife
      descriptors.add(Factory.createPredicate("handle(X)"));
      descriptors.add(Factory.createPredicate("on(X,Y)"));
    } else if (searchNum == 6) {
      descriptors.add(Factory.createPredicate("object(X)"));
      descriptors.add(Factory.createPredicate("grasp_point(Y)"));
      descriptors.add(Factory.createPredicate("near(Y,X)"));
    } else if (searchNum == 7) {
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("knife(Y)")); //knife
      descriptors.add(Factory.createPredicate("blade(W)"));
      descriptors.add(Factory.createPredicate("near(X,W)"));
      descriptors.add(Factory.createPredicate("on(W,Y)"));
    } else if (searchNum == 8) {
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("object(Y)")); //knife
      descriptors.add(Factory.createPredicate("handle(Z)"));
      descriptors.add(Factory.createPredicate("blade(W)"));
      descriptors.add(Factory.createPredicate("near(X,Z)"));
      descriptors.add(Factory.createPredicate("near(X,W)"));
      descriptors.add(Factory.createPredicate("on(Z,Y)"));
      descriptors.add(Factory.createPredicate("on(W,Y)"));
    } else if (searchNum == 9) {
      descriptors.add(Factory.createPredicate("grasp_point(X)"));
      descriptors.add(Factory.createPredicate("knife(Y)")); //knife
      descriptors.add(Factory.createPredicate("blade(Z)"));
      descriptors.add(Factory.createPredicate("partOf(Z,Y)"));
      descriptors.add(Factory.createPredicate("on(X,Z)"));
    }
    Long typeId = component.getTypeId(descriptors);

    MemoryObject token = null;
    double conf = 0.5;
    List<Long> tokenIds = component.getTokenIds(typeId, conf);
    if (tokenIds != null && !tokenIds.isEmpty()) {
      Long tokenId = tokenIds.get(0);
      token = component.getToken(tokenId, conf);
      token.printSceneGraph();
    }

    component.stopType(typeId);

    if (token != null) {
      List<Predicate> constraints = new ArrayList();
      constraints.add(Factory.createPredicate("grasp_point(X)"));
      constraints.add(Factory.createPredicate("handle(Z)"));
//      constraints.add(Factory.createPredicate("blade(W)"));
      constraints.add(Factory.createPredicate("near(X,Z)"));
//      constraints.add(Factory.createPredicate("near(X,W)"));
      Map<Variable, Symbol> bindings = new HashMap<>();
      List<Float> confs = MemoryObjectUtil.getConfidence(token, constraints, bindings);
      log.info("Confidence: " + confs);
    } else {
      log.info("No token found.");
    }
  }
}
