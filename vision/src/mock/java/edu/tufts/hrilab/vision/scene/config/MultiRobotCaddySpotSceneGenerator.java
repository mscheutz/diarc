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

public class MultiRobotCaddySpotSceneGenerator extends SceneGenerator {

  //Collection of all the old predicates and prefabricated objects used in the scene generator before it was split up into multiple files

  @Override
  public SceneCollection generateSceneCollection() {

    Variable x = Factory.createVariable("X", "spotphysobj");
    List<Term> knownProperties = Arrays.asList(
            Factory.createPredicate("any", x),
            Factory.createPredicate("all", x),
            Factory.createPredicate("door", x),
            Factory.createPredicate("handle", x)
    );
    float noise = 0.0f;
    SceneCollection sceneCollection = new SceneCollection(knownProperties, noise);

    // setup scenes

    // scene  0 -- without BandageBox
    Scene scene0 = new Scene("0");
    scene0.addDetectionResult(getDoor());
    scene0.addDetectionResult(getHandle());
    sceneCollection.addScene(scene0);


    return sceneCollection;
  }

  private MemoryObject getDoor() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "spotphysobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate("door", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject getHandle() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "spotphysobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate("handle", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }
}
