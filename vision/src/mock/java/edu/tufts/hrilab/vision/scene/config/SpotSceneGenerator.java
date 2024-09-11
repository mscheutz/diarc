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
            Factory.createPredicate("all", x),
            Factory.createPredicate("unknown", x),
            Factory.createPredicate("apple", x),
            Factory.createPredicate("bottle", x)
    );
    float noise = 0.0f;
    SceneCollection sceneCollection = new SceneCollection(knownProperties, noise);

    // setup scenes

    // scene  0 -- without BandageBox
    Scene scene0 = new Scene("0");
    scene0.addDetectionResult(getObject("apple"));
    scene0.addDetectionResult(getObject("apple"));
    scene0.addDetectionResult(getObject("apple"));
    scene0.addDetectionResult(getObject("apple"));
    scene0.addDetectionResult(getObject("apple"));
    scene0.addDetectionResult(getObject("bottle"));
    scene0.addDetectionResult(getObject("bottle"));
    scene0.addDetectionResult(getObject("bottle"));
    scene0.addDetectionResult(getObject("bottle"));
    scene0.addDetectionResult(getObject("bottle"));
    scene0.addDetectionResult(getObject("bottle"));
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
}
