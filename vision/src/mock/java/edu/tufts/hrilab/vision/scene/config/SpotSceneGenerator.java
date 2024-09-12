/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.scene.config;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.scene.Scene;
import edu.tufts.hrilab.vision.scene.SceneCollection;
import edu.tufts.hrilab.vision.scene.SceneGenerator;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import java.util.Arrays;
import java.util.List;

public class SpotSceneGenerator extends SceneGenerator {

  //Collection of all the old predicates and prefabricated objects used in the scene generator before it was split up into multiple files

  @Override
  public SceneCollection generateSceneCollection() {

    Variable x = Factory.createVariable("X", "physobj");
    List<Term> knownProperties = Arrays.asList(
            Factory.createPredicate("any", x),
            Factory.createPredicate("apple", x),
            Factory.createPredicate("bottle", x),
            Factory.createPredicate("unknown", x),
            Factory.createPredicate("object_consists_of_three_components", x),
            Factory.createPredicate("central_component_is_shaped_like_an_ellipsoid", x),
            Factory.createPredicate("second_component_is_shaped_like_a_cylinder", x),
            Factory.createPredicate("second_component_is_positioned_to_the_right", x),
            Factory.createPredicate("second_component_is_positioned_on_top", x),
            Factory.createPredicate("second_component_is_positioned_centered_along_the_depth_axis", x),
            Factory.createPredicate("second_component_is_of_similar_size", x),
            Factory.createPredicate("third_component_is_shaped_like_a_cylinder", x),
            Factory.createPredicate("third_component_is_positioned_in_front", x),
            Factory.createPredicate("third_component_is_positioned_to_the_left", x),
            Factory.createPredicate("third_component_is_positioned_on_top", x),
            Factory.createPredicate("third_component_is_larger", x)
    );
    float noise = 0.0f;
    SceneCollection sceneCollection = new SceneCollection(knownProperties, noise);

    // setup scenes

    // scene  0 -- without BandageBox
    Scene scene0 = new Scene("0");
    scene0.addDetectionResult(getObject("apple"));
    scene0.addDetectionResult(getObject("bottle"));
    scene0.addDetectionResult(getUnknownObject());
    sceneCollection.addScene(scene0);

    return sceneCollection;
  }

  private MemoryObject getObject(String type) {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate(type, x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject getUnknownObject() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);

    mo.addDescriptor(Factory.createPredicate("unknown", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("object_consists_of_three_components", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("central_component_is_shaped_like_an_ellipsoid", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("second_component_is_shaped_like_a_cylinder", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("second_component_is_positioned_to_the_right", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("second_component_is_positioned_on_top", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("second_component_is_positioned_centered_along_the_depth_axis", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("second_component_is_of_similar_size", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("third_component_is_shaped_like_a_cylinder", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("third_component_is_positioned_in_front", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("third_component_is_positioned_to_the_left", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("third_component_is_positioned_on_top", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("third_component_is_larger", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }
}
