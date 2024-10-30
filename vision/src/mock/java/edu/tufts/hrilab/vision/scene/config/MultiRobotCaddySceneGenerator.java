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

public class MultiRobotCaddySceneGenerator extends SceneGenerator {

    //Collection of all the old predicates and prefabricated objects used in the scene generator before it was split up into multiple files

    @Override
    public SceneCollection generateSceneCollection() {

        Variable x = Factory.createVariable("X", "physobj");
        Variable y = Factory.createVariable("Y", "physobj");
        List<Term> knownProperties = Arrays.asList(
                Factory.createPredicate("any", x),
                Factory.createPredicate("person", x),
                Factory.createPredicate("grasp_point", x),
                Factory.createPredicate("on", x, y),
                Factory.createPredicate("caddy", x),
                Factory.createPredicate("screwbin", x),
                Factory.createPredicate("partOf", x, y),
                Factory.createPredicate("painkiller", x),
                Factory.createPredicate("antiseptic", x),
                Factory.createPredicate("bandagebox", x),
                Factory.createPredicate("gearboxtop", x),
                Factory.createPredicate("gearboxbottom", x),
                Factory.createPredicate("medicalcaddy", x),
                Factory.createPredicate("temi", x)
        );
        float noise = 0.0f;
        SceneCollection sceneCollection = new SceneCollection(knownProperties, noise);

        // setup scenes

        // scene  0 -- without BandageBox
        Scene scene0 = new Scene("0");
        scene0.addDetectionResult(getMedicalCaddy());
//        scene0.addDetectionResult(getBandageBox());
        scene0.addDetectionResult(getAntiseptic());
        scene0.addDetectionResult(getPainkiller());
        sceneCollection.addScene(scene0);

        // scene  1 -- with BandageBox
        Scene scene1 = new Scene("1");
        scene1.addDetectionResult(getMedicalCaddy());
        scene1.addDetectionResult(getBandageBox());
        scene1.addDetectionResult(getAntiseptic());
        scene1.addDetectionResult(getPainkiller());
        sceneCollection.addScene(scene1);

        return sceneCollection;
    }

    private MemoryObject getPainkiller() {
        MemoryObject mo = new MemoryObject();

        Variable x = Factory.createVariable("X", "physobj");

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable(x);
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(Factory.createPredicate("painkiller", x), 0.9f);
//        mo.addDescriptor(Factory.createPredicate("physobj", x), 0.9f);
        mo.setLocation(1,0,1);

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
        mo.setLocation(1,0,1);

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
        mo.setLocation(1,0,1);

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
        mo.setLocation(1,0,1);

        return mo;
    }

    private MemoryObject graspObject() {
        MemoryObject mo = new MemoryObject();

        Variable x = Factory.createVariable("X", "physobj");

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable(x);
        mo.setDetectionConfidence(0.8f);
        mo.setLocation(1,0,1);

        return mo;
    }

}
