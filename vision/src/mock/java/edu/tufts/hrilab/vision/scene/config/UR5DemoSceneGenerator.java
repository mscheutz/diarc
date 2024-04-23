/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.scene.config;

import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.scene.Scene;
import edu.tufts.hrilab.vision.scene.SceneCollection;
import edu.tufts.hrilab.vision.scene.SceneGenerator;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import edu.tufts.hrilab.vision.util.PredicateHelper;

import java.util.Arrays;
import java.util.List;

public class UR5DemoSceneGenerator extends SceneGenerator {
    @Override
    public SceneCollection generateSceneCollection() {
        List<Term> knownProperties = Arrays.asList(
                PredicateHelper.createPredicate("any(X)"),
                PredicateHelper.createPredicate("object(X)"),
                PredicateHelper.createPredicate("jar(X)"),
                PredicateHelper.createPredicate("red(X)"),
                PredicateHelper.createPredicate("purple(X)"),
                PredicateHelper.createPredicate("blue(X)"),
                PredicateHelper.createPredicate("yellow(X)")
                );

        SceneCollection sceneCollection = new SceneCollection(knownProperties, 0.0f);

        // setup scenes
        Scene scene0 = new Scene("0");

        // scene  1
        scene0.addDetectionResult(getRedJar());
        scene0.addDetectionResult(getBlueJar());
        scene0.addDetectionResult(getPurpleJar());
        scene0.addDetectionResult(getYellowJar());
        sceneCollection.addScene(scene0);

        return sceneCollection;
    }

    private MemoryObject getRedJar() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("jar(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("red(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);
        return mo;
    }

    private MemoryObject getBlueJar() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("jar(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("blue(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);
        return mo;
    }

    private MemoryObject getPurpleJar() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("jar(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("purple(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);
        return mo;
    }
    private MemoryObject getYellowJar() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("jar(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("yellow(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);
        return mo;
    }
}
