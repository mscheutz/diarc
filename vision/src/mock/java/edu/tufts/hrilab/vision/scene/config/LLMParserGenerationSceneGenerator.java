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

public class LLMParserGenerationSceneGenerator extends SceneGenerator {

  //Collection of all the old predicates and prefabricated objects used in the scene generator beforeD it was split up into multiple files

  @Override
  public SceneCollection generateSceneCollection() {

    Variable x = Factory.createVariable("X", "physobj");
    Variable y = Factory.createVariable("Y", "physobj");
    List<Term> knownProperties = Arrays.asList(
        Factory.createPredicate("any", x),
        Factory.createPredicate("blue", x),
        Factory.createPredicate("red", x),
        Factory.createPredicate("person", x),
        Factory.createPredicate("grasp_point", x),
        Factory.createPredicate("on", x, y),
        Factory.createPredicate("leftof", x, y),
        Factory.createPredicate("rightof", x, y),
        Factory.createPredicate("in", x, y),
        Factory.createPredicate("caddy", x),
        Factory.createPredicate("screwbin", x),
        Factory.createPredicate("partOf", x, y),
        Factory.createPredicate("painkiller", x),
        Factory.createPredicate("antiseptic", x),
        Factory.createPredicate("bandagebox", x),
        Factory.createPredicate("gearboxtop", x),
        Factory.createPredicate("gearboxbottom", x),
        Factory.createPredicate("block", x),
        Factory.createPredicate("medicalcaddy", x),

        Factory.createPredicate("apple", x),
        Factory.createPredicate("baseball", x),
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
        Factory.createPredicate("box", x)
    );
    float noise = 0.0f;
    SceneCollection sceneCollection = new SceneCollection(knownProperties, noise);

    // setup scenes

    // scene  0 -- without BandageBox
    Scene scene0 = new Scene("0");
    scene0.addDetectionResult(getNewObject("apple"));
    scene0.addDetectionResult(getNewObject("baseball"));
    scene0.addDetectionResult(getNewObject("glassbottle"));
    scene0.addDetectionResult(getNewObject("bowl"));
    scene0.addDetectionResult(getNewObject("carrot"));
    scene0.addDetectionResult(getNewObject("donut"));
    scene0.addDetectionResult(getNewObject("flowerpot"));
    scene0.addDetectionResult(getNewObject("computermouse"));
    scene0.addDetectionResult(getNewObject("car"));
    scene0.addDetectionResult(getNewObject("sportsbottle"));
    scene0.addDetectionResult(getNewObject("teddybear"));
    scene0.addDetectionResult(getNewObject("tennisball"));
    scene0.addDetectionResult(getNewObject("waterbottle"));
    scene0.addDetectionResult(getNewObject("box"));
    scene0.addDetectionResult(getNewObject("painkiller"));
    scene0.addDetectionResult(getNewObject("antiseptic"));
    scene0.addDetectionResult(getNewObject("bandagebox"));
    scene0.addDetectionResult(getNewObject("medicalcaddy"));
    scene0.addDetectionResult(getNewObject("block"));

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
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject getPainkiller() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate("painkiller", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject getBandageBox() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate("bandagebox", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject getMedicalCaddy() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate("medicalcaddy", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject getAntiseptic() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.addDescriptor(Factory.createPredicate("antiseptic", x), 0.9f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

  private MemoryObject graspObject() {
    MemoryObject mo = new MemoryObject();

    Variable x = Factory.createVariable("X", "physobj");

    // instantiate Test MemoryObject
    mo.setTokenId(tokenIdGenerator.getNext());
    mo.setVariable(x);
    mo.setDetectionConfidence(0.8f);
    mo.setLocation(1, 0, 1);

    return mo;
  }

}
