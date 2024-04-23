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

public class MiscSceneGenerator extends SceneGenerator {

    //Collection of all the old predicates and prefabricated objects used in the scene generator before it was split up into multiple files

    @Override
    public SceneCollection generateSceneCollection() {

        List<Term> knownProperties = Arrays.asList(
                PredicateHelper.createPredicate("any(X)"),
                PredicateHelper.createPredicate("object(X)"),
                PredicateHelper.createPredicate("person(X)"),
                PredicateHelper.createPredicate("blade(X)"),
                PredicateHelper.createPredicate("handle(X)"),
                PredicateHelper.createPredicate("knife(X)"),
                PredicateHelper.createPredicate("plate(X)"),
                PredicateHelper.createPredicate("grasp_point(X)"),
                PredicateHelper.createPredicate("lunchbox(X,Y)"),
                PredicateHelper.createPredicate("bill(X)"),
                PredicateHelper.createPredicate("magnet(X)"),
                PredicateHelper.createPredicate("fridge(X)"),
                PredicateHelper.createPredicate("front(X)"),
                PredicateHelper.createPredicate("back(X)"),
                PredicateHelper.createPredicate("medkit(X)"),
                PredicateHelper.createPredicate("cup(X)"),
                PredicateHelper.createPredicate("bowl(X)"),
                PredicateHelper.createPredicate("teabox(X)"),
                PredicateHelper.createPredicate("box(X)"),
                PredicateHelper.createPredicate("jar(X)"),
                PredicateHelper.createPredicate("ball(X)"),
                PredicateHelper.createPredicate("red(X)"),
                PredicateHelper.createPredicate("white(X)"),
                PredicateHelper.createPredicate("blue(X)"),
                PredicateHelper.createPredicate("orange(X)"),
                PredicateHelper.createPredicate("green(X)"),
                PredicateHelper.createPredicate("caddy(X)"),
                PredicateHelper.createPredicate("screwbin(X)"),
                PredicateHelper.createPredicate("partOf(X,Y)"),
                PredicateHelper.createPredicate("screw(X)"),
                PredicateHelper.createPredicate("smallgear(X)"),
                PredicateHelper.createPredicate("largegear(X)"),
                PredicateHelper.createPredicate("gearboxtop(X)"),
                PredicateHelper.createPredicate("gearboxbottom(X)")
        );
        float noise = 0.0f;
        SceneCollection sceneCollection = new SceneCollection(knownProperties, noise);

        // setup scenes
        Scene scene0 = new Scene("0");

        // scene  1
        scene0.addDetectionResult(getKnife());
        scene0.addDetectionResult(getRedTeaBox());
        scene0.addDetectionResult(getGreenTeaBox());
        scene0.addDetectionResult(getPlate());
        scene0.addDetectionResult(getFridge());
        scene0.addDetectionResult(getMagnet());
        sceneCollection.addScene(scene0);

        return sceneCollection;
    }

    private MemoryObject getDracoBox() {
        MemoryObject mo = getBox();
        //Match with the M simulation
        mo.setLocation(0.6, 0.0, 0.2);
//    //TODO set to match with the DART simulation
//    mo.setLocation(-0.3, -0.4, -0.01);
//    MemoryObject mo_grasp_point = new MemoryObject();
//    mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
//    mo_grasp_point.setVariable("W");
//    mo_grasp_point.setDetectionConfidence(0.9f);
//    mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
//    mo.addChild(mo_grasp_point);
//
//    // add grasp point relations
//    Term relation_term = PredicateHelper.createPredicate("on(W,X)");
//    mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getDracoBox2() {
        MemoryObject mo = getBox();
        //Match with the M simulation
        //mo.setLocation(0.8, 0.4, 0.05);
        return mo;
    }

    private MemoryObject getBill() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("bill(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("person(X)"), 0.9f);
        mo.setLocation(2, 1, 1);

        return mo;
    }

    private MemoryObject getPlate() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("plate(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getKnife() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(1.0f);
        mo.addDescriptor(PredicateHelper.createPredicate("knife(X)"), 1);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 1);
        mo.setLocation(1,0,1);

        //////////////////////

        // add handle of object
        MemoryObject mo_handle = new MemoryObject();
        mo_handle.setTokenId(tokenIdGenerator.getNext());
        mo_handle.setVariable("Y");
        mo_handle.setDetectionConfidence(0.9f);
        mo_handle.addDescriptor(PredicateHelper.createPredicate("handle(Y)"), 1);
        mo.addChild(mo_handle);

        // add handle relation to knife
        Term relation_term = PredicateHelper.createPredicate("on(Y,X)");
        mo.addRelation(0.85f, relation_term, mo_handle, true);

        //////////////////////

        // add blade of object
        MemoryObject mo_blade = new MemoryObject();
        mo_blade.setTokenId(tokenIdGenerator.getNext());
        mo_blade.setVariable("Z");
        mo_blade.setDetectionConfidence(0.9f);
        mo_blade.addDescriptor(PredicateHelper.createPredicate("blade(Z)"), 1);
        mo.addChild(mo_blade);

        // add blade relation to knife
        relation_term = PredicateHelper.createPredicate("on(Z,X)");
        mo.addRelation(0.85f, relation_term, mo_blade, true);

        //////////////////////

        // add grasp point on blade
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 1);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        relation_term = PredicateHelper.createPredicate("on(W,Z)");
        mo_blade.addRelation(0.85f, relation_term, mo_grasp_point, true);

        //////////////////////

        // add grasp point on handle
        mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 1);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        relation_term = PredicateHelper.createPredicate("on(W,Y)");
        mo_handle.addRelation(0.8f, relation_term, mo_grasp_point, true);

        //////////////////////

        // add grasp point on the whole knife
        mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 1f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }


    private MemoryObject getMug() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("mug(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);

        //////////////////////

        // add handle of object
        MemoryObject mo_handle = new MemoryObject();
        mo_handle.setTokenId(tokenIdGenerator.getNext());
        mo_handle.setVariable("Y");
        mo_handle.setDetectionConfidence(0.9f);
        mo_handle.addDescriptor(PredicateHelper.createPredicate("handle(Y)"), 0.9f);
        mo.addChild(mo_handle);

        // add handle relation to knife
        Term relation_term = PredicateHelper.createPredicate("on(Y,X)");
        mo.addRelation(0.85f, relation_term, mo_handle, true);

        //////////////////////

        // add grasp point on handle
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        relation_term = PredicateHelper.createPredicate("on(W,Y)");
        mo_handle.addRelation(0.8f, relation_term, mo_grasp_point, true);

        //////////////////////

        // add grasp point on the whole mug
        mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo_handle.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getMedkit() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("medkit(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getBall() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("ball(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }


    private MemoryObject getCup(String color) {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("cup(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);
        if(!color.equals("")){
            mo.addDescriptor(PredicateHelper.createPredicate(color+"(X)"), 0.9f);
        }


        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getBowl() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("bowl(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getBox() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("box(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getGreenTeaBox() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("teabox(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("green(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("box(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);
        return mo;
    }

    private MemoryObject getFridge() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("fridge(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("front(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getMagnet() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("magnet(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("back(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getRedTeaBox() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("teabox(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("box(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("red(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        return mo;
    }

    private MemoryObject getRedBall() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("ball(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("red(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);
        return mo;
    }

    private MemoryObject getScrewBin() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("Y");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("caddy(Y)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject moScrew = new MemoryObject();
        moScrew.setTokenId(tokenIdGenerator.getNext());
        moScrew.setVariable("X");
        moScrew.setDetectionConfidence(0.9f);
        moScrew.addDescriptor(PredicateHelper.createPredicate("screwbin(X)"), 0.9f);
        mo.addChild(moScrew);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("partOf(X,Y)");
        mo.addRelation(0.85f, relation_term, moScrew, true);
        return mo;
    }

    private MemoryObject getScrew() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("screw(X)"), 0.9f);
        mo.setLocation(1,0,1);

        return mo;
    }

    private MemoryObject getJar() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("jar(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,1,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.95f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getContainerB() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("containerB(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getContainerC() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("containerC(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getScale() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("scale(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject getlunchbox() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("lunchbox(X)"), 0.9f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        // add grasp point on the whole mug
        MemoryObject mo_grasp_point = new MemoryObject();
        mo_grasp_point.setTokenId(tokenIdGenerator.getNext());
        mo_grasp_point.setVariable("W");
        mo_grasp_point.setDetectionConfidence(0.9f);
        mo_grasp_point.addDescriptor(PredicateHelper.createPredicate("grasp_point(W)"), 0.9f);
        mo.addChild(mo_grasp_point);

        // add grasp point relations
        Term relation_term = PredicateHelper.createPredicate("on(W,X)");
        mo.addRelation(0.85f, relation_term, mo_grasp_point, true);
        return mo;
    }

    private MemoryObject graspObject() {
        MemoryObject mo = new MemoryObject();

        // instantiate Test MemoryObject
        mo.setTokenId(tokenIdGenerator.getNext());
        mo.setVariable("X");
        mo.setDetectionConfidence(0.8f);
        mo.addDescriptor(PredicateHelper.createPredicate("object(X)"), 0.9f);
        mo.setLocation(1,0,1);

        return mo;
    }

}
