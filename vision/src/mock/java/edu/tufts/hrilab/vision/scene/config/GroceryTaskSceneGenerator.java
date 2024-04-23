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

public class GroceryTaskSceneGenerator extends SceneGenerator {

  //Collection of all the old predicates and prefabricated objects used in the scene generator before it was split up into multiple files

  @Override
  public SceneCollection generateSceneCollection() {

    Variable x = Factory.createVariable("X", "physobj");
    Variable y = Factory.createVariable("Y");
    List<Term> knownProperties = Arrays.asList(
            Factory.createPredicate("grasp_point", x),
            Factory.createPredicate("physobj", x),
            Factory.createPredicate("on", y, x),
            Factory.createPredicate("apple", x),
            Factory.createPredicate("grocery", x),
            Factory.createPredicate("glassbottle", x),
            Factory.createPredicate("bowl", x),
            Factory.createPredicate("carrot", x),
            Factory.createPredicate("donut", x),
            Factory.createPredicate("flowerpot", x),
            Factory.createPredicate("computermouse", x),
            Factory.createPredicate("car", x),
            Factory.createPredicate("sportsbottle", x),
            Factory.createPredicate("teddybear", x),
            Factory.createPredicate("tennisball", x),
            Factory.createPredicate("waterbottle", x),
            Factory.createPredicate("compa", x),
            Factory.createPredicate("compb", x),
            Factory.createPredicate("compc", x)
    );
    float noise = 0.0f;
    SceneCollection sceneCollection = new SceneCollection(knownProperties, noise);

    // setup scenes

    // scene  0 -- without BandageBox
    Scene scene0 = new Scene("0");
    scene0.addDetectionResult(getNewGrocery("apple", 0, 1));
    scene0.addDetectionResult(getNewGrocery("apple", 0, 1));
    scene0.addDetectionResult(getNewGrocery("apple", 0, 1));
    scene0.addDetectionResult(getNewGrocery("carrot", 1, 0));
    scene0.addDetectionResult(getNewGrocery("carrot", 1, 0));
    scene0.addDetectionResult(getNewGrocery("carrot", 1, 0));
    scene0.addDetectionResult(getNewGrocery("donut", 1, 1));
    scene0.addDetectionResult(getNewGrocery("donut", 1, 1));
    scene0.addDetectionResult(getNewGrocery("donut", 1, 1));
    scene0.addDetectionResult(getNewObject("compa"));
    scene0.addDetectionResult(getNewObject("compb"));
    scene0.addDetectionResult(getNewObject("compc"));

    sceneCollection.addScene(scene0);

    return sceneCollection;
  }


  private MemoryObject getNewObject(String objName) {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate(objName, x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("physobj", x), 0.9f);
    mo.setLocation(0, 0, 1);

    return mo;
  }

  private MemoryObject getNewGrocery(String objName) {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate(objName, x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("grocery", x), 0.9f);
    mo.addDescriptor(Factory.createPredicate("physobj", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject getNewGrocery(String objName, int x, int y) {
    MemoryObject mo = new MemoryObject();

    Variable var = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(var);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate(objName, var), 0.9f);
    mo.addDescriptor(Factory.createPredicate("grocery", var), 0.9f);
    mo.addDescriptor(Factory.createPredicate("physobj", var), 0.9f);
    mo.setLocation(x, y, 1);

    return mo;
  }
}
