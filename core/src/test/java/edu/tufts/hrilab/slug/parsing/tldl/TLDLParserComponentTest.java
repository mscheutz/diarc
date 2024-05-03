/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.slug.parsing.tldl;

import java.util.Arrays;
import java.util.List;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.rules.TestRule;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

import static org.junit.Assert.assertTrue;

public class TLDLParserComponentTest {
    private TLDLParserComponent component;
    private static Logger log = LoggerFactory.getLogger(edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponentTest.class);

    @Rule
    public TestRule watcher = new TestWatcher() {
        protected void starting(Description description) {
            log.info(description.getMethodName());
        }
    };


    public TLDLParserComponentTest() {
        component = DiarcComponent.createInstance(TLDLParserComponent.class, "");
    }

    //brad this is like this so I don't have to change a bunch of old tests, it should probably be updated at some point
    private boolean testUtterance(String utterance, String desired, String speaker, String listener) {
        // component.addWords(complete);
        Utterance parseResult = component.parseUtterance(new Utterance(Factory.createSymbol(speaker),
                Factory.createSymbol(listener),
                Arrays.asList(utterance.split(" ")),
                UtteranceType.UNKNOWN,
                true));

        String outcome = parseResult.toString();
        if (outcome.equals(desired)) {
            log.info("outcome: " + outcome);
            log.info("desired: " + desired + "\n");
        } else {
            log.error("outcome: " + outcome);
            log.error("desired: " + desired + "\n");
        }
        return outcome.equals(desired);
    }

    //TODO:brad: see the comments in the body of this regarding refactoring the action learning/nl pipeline
    @Test
    public void twoNaosTest() {
        log.info("twoNaosTest");
        component.addDictionary("templatedict.dict");
        component.addDictionary("templatedictLearned.dict");

        assertTrue(testUtterance(
                "hello shafer",
                "GREETING(commX,shafer,hello,{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("shafer could you please stand",
                "QUESTION(commX,shafer,can(stand(shafer)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("please walk forward",
                "INSTRUCT(commX,shafer,move(shafer,forward),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("do you trust me shafer",
                "QUESTION(commX,shafer,trust(shafer,commX),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("the obstacle is not solid",
                "STATEMENT(commX,shafer,notPropertyOf(obstacle,solid),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("now walk forward",
                "INSTRUCT(commX,shafer,move(shafer,forward),{})",
                "commX",
                "self"
        ));


        assertTrue(testUtterance("stop",
                "INSTRUCT(commX,shafer,stop(shafer),{})",
                "commX",
                "self"
        ));


        assertTrue(testUtterance("could you please walk backward",
                "QUESTION(commX,shafer,can(move(shafer,back)),{})",
                "commX",
                "self"
        ));


        assertTrue(testUtterance("the area behind you is safe",
                "STATEMENT(commX,shafer,propertyOf(behind(area,shafer),safe),{})",
                "commX",
                "self"
        ));

        //semantics.add("STATEMENT(commX,shafer,propertyOf(area(behind(shafer)),safe),{})",

        assertTrue(testUtterance("walk backward",
                "INSTRUCT(commX,shafer,move(shafer,back),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("stop",
                "INSTRUCT(commX,shafer,stop(shafer),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("turn right",
                "INSTRUCT(commX,shafer,turn(shafer,right),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("could you turn right",
                "QUESTION(commX,shafer,can(turn(shafer,right)),{})",
                "commX",
                "self"
        ));
        //semantics.add("INSTRUCT(commX,shafer,move(right),{})");

        assertTrue(testUtterance("can you please turn right",
                "QUESTION(commX,shafer,can(turn(shafer,right)),{})",
                "commX",
                "self"
        ));
        //semantics.add("INSTRUCT(commX,shafer,move(right),{})");

        assertTrue(testUtterance("stop",
                "INSTRUCT(commX,shafer,stop(shafer),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("walk forward",
                "INSTRUCT(commX,shafer,move(shafer,forward),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("do you trust me shafer",
                "QUESTION(commX,shafer,trust(shafer,commX),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("i will catch you",
                "STATEMENT(commX,shafer,will(catch(commX,shafer)),{})",
                "commX",
                "self"
        ));


        assertTrue(testUtterance("can you please walk forward",
                "QUESTION(commX,shafer,can(move(shafer,forward)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("shafer please stand",
                "INSTRUCT(commX,shafer,stand(shafer),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("can you stand",
                "QUESTION(commX,shafer,can(stand(shafer)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("can dempster stand",
                "QUESTION(commX,shafer,can(stand(dempster)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("could you do a squat",
                "QUESTION(commX,shafer,can(doasquat(shafer)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("could you please do a squat",
                "QUESTION(commX,shafer,can(doasquat(shafer)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("ok relax",
                "INSTRUCT(commX,shafer,rest(shafer),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("dempster please stand",
                "INSTRUCT(commX,dempster,stand(dempster),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("shafer tell dempster to walk forward",
                "INSTRUCT(commX,shafer,tell(shafer,to(move(dempster,forward))),{})",
                "commX",
                "self"
        ));

        //TODO:brad: add an example of tell that here?

        assertTrue(testUtterance("do a squat dempster",
                "INSTRUCT(commX,dempster,doasquat(dempster),{})",
                "commX",
                "self"
        ));



        assertTrue(testUtterance("dance",
                "INSTRUCT(commX,dempster,dance(dempster),{})",
                "commX",
                "self"
        ));

        //TODO:brad: I think the on the fly learning that we used to do here is pretty hard to do in general, and we would be better served by having this fail here, and having a good failure explanation that is something like I don't know what "dance means"
//        assertTrue(testUtterance("dance with shafer and andy",
//                "INSTRUCT(commX,dempster,dance(dempster,shafer,andy),{})",
//                "commX",
//                "self"
//        ));

        assertTrue(testUtterance("dance with shafer",
                "INSTRUCT(commX,dempster,dance(dempster,shafer),{})",
                "commX",
                "self"
        ));


        //TODO:brad: the above begs the question, of how we determine the predicate structure used in this teaching
        // I think the best we can do is an arbitrary string here, and then perhaps to infer the semantics from the
        // learned action? and use that to update the dictionary after?
        // That could be a good start a generating parse rules directly from action signatures.
//        assertTrue(testUtterance("I will teach you how to dance with shafer and andy",
//                "STATEMENT(commX,dempster,will(teach(commX,dempster,to(dance(dempster,shafer,andy)))),{})",
//                "commX",
//                "self"
//        ));

        assertTrue(testUtterance("I will teach you how to dance with shafer",
                "STATEMENT(commX,dempster,will(teach(commX,dempster,to(dance(dempster,shafer)))),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("I will teach you how to do a squat",
                "STATEMENT(commX,dempster,will(teach(commX,dempster,to(doasquat(dempster)))),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("raise your hands",
                "INSTRUCT(commX,dempster,raise(dempster,hands),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("crouch down",
                "INSTRUCT(commX,dempster,crouchDown(dempster),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("stand up",
                "INSTRUCT(commX,dempster,standUp(dempster),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("lower your hands",
                "INSTRUCT(commX,dempster,lower(dempster,hands),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("that is how you do a squat",
                "STATEMENT(commX,dempster,endTeaching(dempster,doasquat(dempster)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("do a squat Dempster",
                "INSTRUCT(commX,dempster,doasquat(dempster),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("please do a squat",
                "INSTRUCT(commX,dempster,doasquat(dempster),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Shafer do a squat",
                "INSTRUCT(commX,shafer,doasquat(shafer),{})",
                "commX",
                "self"
        ));


        assertTrue(testUtterance("does dempster see an obstacle",
                "QUESTION(commX,shafer,see(dempster,obstacle),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("does Dempster trust me",
                "QUESTION(commX,shafer,trust(dempster,commX),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("do you see support",
                "QUESTION(commX,shafer,see(shafer,support),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("do you see an obstacle shafer",
                "QUESTION(commX,shafer,see(shafer,obstacle),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("dempster tell shafer to walk forward",
                "INSTRUCT(commX,dempster,tell(dempster,to(move(shafer,forward))),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("thank you dempster",
                "GREETING(commX,dempster,thank_you,{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("thank you shafer",
                "GREETING(commX,shafer,thank_you,{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("dempster tell shafer to do a squat",
                "INSTRUCT(commX,dempster,tell(dempster,to(doasquat(shafer))),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("shafer tell dempster to stand",
                "INSTRUCT(commX,shafer,tell(shafer,to(stand(dempster))),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("shafer tell dempster to crouch",
                "INSTRUCT(commX,shafer,tell(shafer,to(crouch(dempster))),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("the area behind dempster is safe",
                "STATEMENT(commX,shafer,propertyOf(behind(area,dempster),safe),{})",
                "commX",
                "self"
        ));
        //semantics.add("STATEMENT(commX,shafer,propertyOf(area(behind(dempster)),safe),{})",

        assertTrue(testUtterance("dempster tell shafer to stop",
                "INSTRUCT(commX,dempster,tell(dempster,to(stop(shafer))),{})",
                "commX",
                "self"
        ));

//    testUtterance("I want shafer to turn left");
//    semantics.add("STATEMENT(commX,dempster,want(commX,turn(shafer,left)),{},{})");
//
//    testUtterance("I need you to sit");
//    semantics.add("STATEMENT(commX,dempster,want(commX,sit(dempster)),{},{})");
//
//    testUtterance("Shafer needs you to crouch");
//    semantics.add("STATEMENT(commX,dempster,want(shafer,crouch(dempster)),{},{})");

        assertTrue(testUtterance("please sit",
                "INSTRUCT(commX,dempster,sit(dempster),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("could you please sit",
                "QUESTION(commX,dempster,can(sit(dempster)),{})",
                "commX",
                "self"
        ));
        assertTrue(testUtterance("I will catch you",
                "STATEMENT(commX,dempster,will(catch(commX,dempster)),{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("forget that I will catch you",
                "INSTRUCT(commX,dempster,forgetThat(dempster,will(catch(commX,dempster))),{})",
                "commX",
                "self"
        ));
        //semantics.add("INSTRUCT(commX,dempster,forgetThat(dempster,catch(commX,dempster)),{})",

        assertTrue(testUtterance("forget how to do a squat",
                "INSTRUCT(commX,dempster,forgetHow(dempster,doasquat(dempster)),{})",
                "commX",
                "self"
        ));

//    testUtterance("it is unsafe to walk forward if there is an obstacle in front of you");
//    semantics.add("STATEMENT(commX,dempster,hornClause(unsafe(move(dempster,straight)),inFront(obstacle,dempster)),{},{})");
//
//    testUtterance("it is safe to walk forward when there is an obstacle in front of you if the obstacle is not solid");
//    semantics.add("STATEMENT(commX,dempster,hornClause(safe(move(dempster,forward)),and(inFront(dempster,obstacle),not(solid(obstacle))))),{},{})");

//    testUtterance("The obstacle in front of you is not solid");
//     semantics.add("STATEMENT(commX,dempster,propertyOf(inFront(dempster,obstacle),not(solid))),{},{})");
//
//    testUtterance("It is unsafe to walk forward if there is no support in front of you");
//    semantics.add("STATEMENT(commX,dempster,hornClause((unsafe(move(dempster,forward)))),inFront(no(support),dempster)),{},{})");
//
//    testUtterance("It is safe to walk forward when there is no support in front of you if someone will catch you");
//    semantics.add("STATEMENT(commX,dempster,hornClause(safe(move(dempster,forward)),and(inFront(no(support),dempster),willCatch(agent,you)))),{},{})");
//
//    testUtterance("I will catch you");
//     semantics.add("STATEMENT(commX,dempster,willCatch(commX,dempster)),{},{})");


//    testUtterance("shafer forget how to do a squat");
//    semantics.add("INSTRUCT(commX,shafer,forgetHow(shafer,how(doasquat(shafer))),{},{})");

//    assertTrue(testUtterances(text, semantics));
    }

    @Test //TODO: Try with different dictionary or delete
    @Ignore
    public void lgosl() {
        component.addDictionary("Demos.dict");
//    component.setActor("shafer");
//    ArrayList<String> text = new ArrayList<>();
//    ArrayList<String> semantics = new ArrayList<>();

        assertTrue(testUtterance("Do you see a knife",
                "QUESTION(commX,self,see(self,knife),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Do you see an object",
                "QUESTION(commX,self,see(self,object),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Do you see a unk0",
                "QUESTION(commX,self,see(self,unk0),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Do you see a unk1",
                "QUESTION(commX,self,see(self,unk1),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Pick up the knife by the unk0",
                "INSTRUCT(commX,self,pickUp(self,partOf(unk0,knife)),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Pick up the knife by the unk1",
                "INSTRUCT(commX,self,pickUp(self,partOf(unk1,knife)),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Pick up the knife by the handle",
                "INSTRUCT(commX,self,pickUp(self,partOf(handle,knife)),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Pick up the knife by the blade",
                "INSTRUCT(commX,self,pickUp(self,partOf(blade,knife)),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Pick up the knife",
                "INSTRUCT(commX,self,pickUp(self,knife),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Pick up the object",
                "INSTRUCT(commX,self,pickUp(self,object),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("The orange part of the knife is the unk0",
                "STATEMENT(commX,self,definitionOf(partOf(orange,knife),unk0),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("The gray part of the knife is the unk1",
                "STATEMENT(commX,self,definitionOf(partOf(gray,knife),unk1),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("The orange part of the knife is the handle",
                "STATEMENT(commX,self,definitionOf(partOf(orange,knife),handle),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("The gray part of the knife is the blade",
                "STATEMENT(commX,self,definitionOf(partOf(gray,knife),blade),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Point to the knife",
                "INSTRUCT(commX,self,pointTo(self,knife),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Point to the object",
                "INSTRUCT(commX,self,pointTo(self,object),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Hand it to me",
                "INSTRUCT(commX,self,handOver(self,commX,lastobject),{},{})",
                "commX",
                "self"
        ));

        assertTrue(testUtterance("Start over",
                "INSTRUCT(commX,self,goToStartPose(false),{},{})",
                "commX",
                "self"
        ));

//    assertTrue(testUtterances(text, semantics));
    }

    @Test
    public void PR2DemosTest() {
        log.info("PR2DemosTest");
        component.addDictionary("templatedict.dict");
        component.addDictionary("templatedictLearned.dict");

        assertTrue(testUtterance("Do you see the object",
                "QUESTION(commX,shafer,see(shafer,VAR0),{object(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("This object is a plate",
                "STATEMENT(commX,shafer,instanceOf(VAR0,plate),{object(VAR0),REFACTIVATED(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("pass me the plate",
                "INSTRUCT(commX,shafer,pass(shafer,commX,VAR0),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("Okay I will teach you how to pass me the plate",
                "STATEMENT(commX,shafer,will(teach(commX,shafer,to(pass(shafer,commX,VAR0)))),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("pickup the plate",
                "INSTRUCT(commX,shafer,pickup(shafer,VAR0),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("Okay I will teach you how to pickup the plate",
                "STATEMENT(commX,shafer,will(teach(commX,shafer,to(pickup(shafer,VAR0)))),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("First find the plate",
                "INSTRUCT(commX,shafer,findObject(shafer,VAR0),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("Then grab the plate",
                "INSTRUCT(commX,shafer,graspObject(shafer,VAR0),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("move the plate up",
                "INSTRUCT(commX,shafer,moveObject(shafer,VAR0,up),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("That is how you pickup a plate",
                "STATEMENT(commX,shafer,endTeaching(shafer,pickup(shafer,VAR0)),{plate(VAR0),INDEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("now move the plate forward",
                "INSTRUCT(commX,shafer,moveObject(shafer,VAR0,forward),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("now move the plate toward me",
                "INSTRUCT(commX,shafer,moveObject(shafer,VAR0,toward,commX),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("move it toward me",
                "INSTRUCT(commX,shafer,moveObject(shafer,VAR0,toward,commX),{it(VAR0),INFOCUS(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("I will grab it",
                "STATEMENT(commX,shafer,will(graspObject(commX,VAR0)),{it(VAR0),INFOCUS(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("Then release the plate",
                "INSTRUCT(commX,shafer,releaseObject(shafer,VAR0),{plate(VAR0),DEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("That is how you pass me a plate",
                "STATEMENT(commX,shafer,endTeaching(shafer,pass(shafer,commX,VAR0)),{plate(VAR0),INDEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("Do you see an object",
                "QUESTION(commX,shafer,see(shafer,VAR0),{object(VAR0),INDEFINITE(VAR0)})",
                "commX",
                "shafer"
        ));

        assertTrue(testUtterance("This object is a knife",
                "STATEMENT(commX,shafer,instanceOf(VAR0,knife),{object(VAR0),REFACTIVATED(VAR0)})",
                "commX",
                "shafer"
        ));

//    testUtterance("The gray part of the knife is the blade");
//    semantics.add("STATEMENT(commX,shafer,definitionOf(partOf(gray,knife),blade),{},{})");
//
//    testUtterance("The orange part of the knife is the handle");
//    semantics.add("STATEMENT(commX,shafer,definitionOf(partOf(orange,knife),handle),{},{})");
//
//    testUtterance("pass me the knife by the blade");
//    semantics.add("INSTRUCT(commX,shafer,pass(shafer,commX,partOf(blade,knife)),{},{})");

//    testUtterance("A knife is used for cutting");
//    semantics.add("STATEMENT(commX,shafer,is(knife, usedFor(eating)),{},{})");

//    testUtterance("To pickup a knife grasp the knife by the handle");
//    semantics.add("STATEMENT(commX,shafer,implies(pickup(shafer,knife),grasp(shafer,partOf(handle,knife))))");
//
//    testUtterance("To pass a knife grasp the knife by the blade");
//    semantics.add("STATEMENT(commX,shafer,implies(pass(shafer,knife),grasp(shafer,partOf(blade,knife))))");

//    testUtterance("pass me something used for cutting");
//    semantics.add("INSTRUCT(commX,shafer,pass(shafer,commX,afforadanceOf(cutting,something)),{},{})");
//    semantics.add("INSTRUCT(commX,shafer,hack(unk1(shafer,unk3),partOf(unk4)),{},{})");
//    H: "hand me the knife by the handle"
//    H: "the orange part of the knife is the handle"

//    assertTrue(testUtterances(text, semantics));

    }

    @Test
    public void performanceAssessmentTest() {
        log.info("performanceAssessmentTest");
        component.addDictionary("templatedict.dict");
        component.addDictionary("multiRobotCaddyGenerated.dict");

        assertTrue(testUtterance("what is the probability that you can assemble the caddy",
                "QUESTION(brad,shafer,probabilityOf(assemble(shafer,VAR0)),{caddy(VAR0),DEFINITE(VAR0)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("how long will it take to do that",
                "QUESTION(brad,shafer,expected(timeTo(VAR0:dialog)),{dothat(VAR0:dialog),ACTIVATED(VAR0:dialog)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("where do you foresee issues",
                "QUESTION(brad,shafer,where(foresee(shafer,issues)),{})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("what is the probability that you can assemble the caddy if you do not fetch the screw from the screw location",
                "QUESTION(brad,shafer,if(modify(delete(fetch(shafer,VAR1:physobj,VAR2))),probabilityOf(assemble(shafer,VAR0))),{caddy(VAR0),screw(VAR1:physobj),screwlocation(VAR2),DEFINITE(VAR0),DEFINITE(VAR1:physobj),DEFINITE(VAR2)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("what is the probability that you can complete the task to assemble the caddy",
                "QUESTION(brad,shafer,probabilityOf(completeTask(assemble(shafer,VAR0))),{caddy(VAR0),DEFINITE(VAR0)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("what is the probability that you can complete the task to assemble the caddy if you successfully fetch the small gear from the small gear location",
                "QUESTION(brad,shafer,if(complete(fetch(shafer,VAR1:physobj,VAR2)),probabilityOf(completeTask(assemble(shafer,VAR0)))),{caddy(VAR0),smallgear(VAR1:physobj),smallgearlocation(VAR2),DEFINITE(VAR0),DEFINITE(VAR1:physobj),DEFINITE(VAR2)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("what would have been the probability that you could complete the task to assemble the caddy if you were holding the screw",
                "QUESTION(brad,shafer,if(state(holding(shafer,VAR1:physobj,arm)),past(probabilityOf(completeTask(assemble(shafer,VAR0))))),{caddy(VAR0),screw(VAR1:physobj),DEFINITE(VAR0),DEFINITE(VAR1:physobj)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("how long would it have taken to do that",
                "QUESTION(brad,shafer,past(expected(timeTo(remaining,VAR0:dialog))),{dothat(VAR0:dialog),ACTIVATED(VAR0:dialog)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("which step of that is most likely to fail",
                "QUESTION(brad,shafer,propertyOf(stepOf(WHAT,VAR0),mostLikely(to(fail(shafer)))),{that(VAR0),ACTIVATED(VAR0)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("what remaining step of assemble the caddy is most likely to fail",
                "QUESTION(brad,shafer,propertyOf(remaining(stepOf(WHAT,assemble(shafer,VAR0))),mostLikely(to(fail(shafer)))),{caddy(VAR0),DEFINITE(VAR0)})",
                "brad",
                "shafer"
        ));

//        assertTrue(testUtterance("how long would it have taken to do that if you fetch the small gear from the small gear location instead of fetch the large gear from the large gear location",
//                "QUESTION(brad,shafer,if(modify(replace(fetch(shafer,VAR3:physobj,VAR4),fetch(shafer,VAR1:physobj,VAR2))),past(expected(timeTo(remaining,VAR0:dialog)))),{dothat(VAR0:dialog),smallgear(VAR1:physobj),smallgearlocation(VAR2),largegear(VAR3:physobj),largegearlocation(VAR4),ACTIVATED(VAR0:dialog),DEFINITE(VAR1:physobj),DEFINITE(VAR2),DEFINITE(VAR3:physobj),DEFINITE(VAR4)})",
//                "brad",
//                "shafer"
//        ));

        assertTrue(testUtterance("how long would it have taken to do that if instead of you fetch the small gear from the small gear location you fetch the large gear from the large gear location",
                "QUESTION(brad,shafer,if(modify(replace(fetch(shafer,VAR3:physobj,VAR4),fetch(shafer,VAR1:physobj,VAR2))),past(expected(timeTo(remaining,VAR0:dialog)))),{dothat(VAR0:dialog),smallgear(VAR1:physobj),smallgearlocation(VAR2),largegear(VAR3:physobj),largegearlocation(VAR4),ACTIVATED(VAR0:dialog),DEFINITE(VAR1:physobj),DEFINITE(VAR2),DEFINITE(VAR3:physobj),DEFINITE(VAR4)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("what step of that is most likely to fail if you do not fetch the small gear from the small gear location",
                "QUESTION(brad,shafer,if(modify(delete(fetch(shafer,VAR1:physobj,VAR2))),propertyOf(stepOf(WHAT,VAR0),mostLikely(to(fail(shafer))))),{that(VAR0),smallgear(VAR1:physobj),smallgearlocation(VAR2),ACTIVATED(VAR0),DEFINITE(VAR1:physobj),DEFINITE(VAR2)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("what step of that would have been most likely to fail if you were holding the screw",
                "QUESTION(brad,shafer,if(state(holding(shafer,VAR1:physobj,arm)),past(propertyOf(stepOf(WHAT,VAR0),mostLikely(to(fail(shafer)))))),{that(VAR0),screw(VAR1:physobj),ACTIVATED(VAR0),DEFINITE(VAR1:physobj)})",
                "brad",
                "shafer"
        ));
    }

    //TODO:brad: see the comments in the body  twonaos test regarding refactoring the action learning/nl pipeline
    @Test
    public void functionalityTest() {
        log.info("functionalityTest");
        component.addDictionary("templatedict.dict");
        component.addDictionary("templatedictLearned.dict");
//        component.setLearn(true);
//    component.setActor("shafer");
//    component.setInteractor("brad");

        assertTrue(testUtterance("do you trust me shafer",
                "QUESTION(brad,shafer,trust(shafer,brad),{})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("who do you trust",
                "QUESTION(brad,shafer,trust(shafer,WHO),{})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("do you trust ravenna",
                "QUESTION(brad,shafer,trust(shafer,ravenna),{})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("does andy trust ravenna",
                "QUESTION(brad,shafer,trust(andy,ravenna),{})",
                "brad",
                "shafer"
        ));
//
//    testUtterance("I will grab the knife");
//    semantics.add("STATEMENT(brad,shafer,will(brad,graspObject(VAR0)),{knife(VAR0),DEFINITE(VAR0)})");
//    //semantics.add("[will(brad,did(brad,graspObject(VAR0))), knife(VAR0)] . [DEFINITE(VAR0)]");
//
//    testUtterance("I will catch you");
//    semantics.add("STATEMENT(brad,shafer,will(brad,catch(shafer)),{})");

//    testUtterance("makesure to putdown the cuttingboard before you cut the tomato");
//    semantics.add("INSTRUCT(brad,shafer,modifyAction(insert(putDown(VAR0)),before(putDown(VAR1)), cut(self,VAR1)),{cuttingBoard(VAR0).tomato(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})");


//    testUtterance("when you cut a tomato make sure you putdown a cuttingboard before you putdown the tomato");
//    semantics.add("INSTRUCT(brad,shafer,modifyAction(cut(shafer,VAR0),insert(putDown(shafer,VAR1)),before(putDown(shafer,VAR2))),{tomato(VAR0),cuttingBoard(VAR1),tomato(VAR2),INDEFINITE(VAR0),INDEFINITE(VAR1),DEFINITE(VAR2)})");

        assertTrue(testUtterance("how would you do a squat",
                "QUESTION(brad,shafer,how(shafer,doasquat(shafer)),{})",
                "brad",
                "shafer"
        ));

//    testUtterance("how do you do a squat");
//    semantics.add("QUESTION(brad,shafer,how(shafer,doasquat()),{})");

        assertTrue(testUtterance("Dempster crouch",
                "INSTRUCT(brad,dempster,crouch(dempster),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("hello andy",
                "GREETING(brad,andy,hello,{})",
                "brad",
                "self"
        ));

        //TODO:brad: this is another case where we can't infer the verb form in general. Despite this case looking simple
        assertTrue(testUtterance("do you know how to nod",
                "QUESTION(brad,andy,knowHow(andy,to(nod(andy))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("execute while learning",
                "INSTRUCT(brad,andy,changeLearningExecution(andy,execute),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you do a squat make sure you lower your arms after you stand up",
                "INSTRUCT(brad,andy,modifyAction(andy,doasquat(andy),insert(lower(andy,arms)),after(standUp(andy))),{})",
                "brad",
                "self"
        ));
//    semantics.add("INSTRUCT(brad,andy,modifyAction(doasquat(andy),insert(lower(andy,arms)),after(standUp(andy))),{})",

        assertTrue(testUtterance("describe how to do a squat",
                "INSTRUCT(brad,andy,describe(andy,how(to(doasquat(andy)))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you do a squat make sure you nod after you stand up",
                "INSTRUCT(brad,andy,modifyAction(andy,doasquat(andy),insert(nod(andy)),after(standUp(andy))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you do a squat make sure you nod after you crouch",
                "INSTRUCT(brad,andy,modifyAction(andy,doasquat(andy),insert(nod(andy)),after(crouch(andy))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you walk forward make sure you turn right after turn left",
                "INSTRUCT(brad,andy,modifyAction(andy,move(andy,forward),insert(turn(andy,right)),after(turn(andy,left))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you walk forward make sure you turn right after you turn left",
                "INSTRUCT(brad,andy,modifyAction(andy,move(andy,forward),insert(turn(andy,right)),after(turn(andy,left))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you nod make sure you nod after you look down",
                "INSTRUCT(brad,andy,modifyAction(andy,nod(andy),insert(nod(andy)),after(look(andy,down))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you walk forward don't turn right after you turn left",
                "INSTRUCT(brad,andy,modifyAction(andy,move(andy,forward),remove(turn(andy,right)),after(turn(andy,left))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you nod don't crouch down after you crouch down",
                "INSTRUCT(brad,andy,modifyAction(andy,nod(andy),remove(crouchDown(andy)),after(crouchDown(andy))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you nod replace turn right with look down",
                "INSTRUCT(brad,andy,modifyAction(andy,nod(andy),replace(with(look(andy,down),turn(andy,right))),none()),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("when you nod replace turn right with look down after stand up",
                "INSTRUCT(brad,andy,modifyAction(andy,nod(andy),replace(with(look(andy,down),turn(andy,right))),after(standUp(andy))),{})",
                "brad",
                "self"
        ));

        //TODO:brad: determine which of these case are relvant, and which should be removed.

/*
        assertTrue(testUtterance("nod is like dance replace dance with look down after dance",
                "INSTRUCT(brad,andy,modifyAction(andy,like(nod(andy),dance(andy)),replace(with(look(andy,down),dance(andy))),after(dance(amdy))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("nod is like dance replace turn right with look down after stand up",
                "INSTRUCT(brad,andy,modifyAction(andy,like(nod(andy),dance(andy)),replace(with(look(andy,down),turn(andy,right))),after(standUp(andy))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("pickup the plate",
                "INSTRUCT(brad,andy,pickup(andy,VAR0),{plate(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("look down dempster",
                "INSTRUCT(brad,dempster,look(dempster,down),{})",
                "brad",
                "self"
        ));

//    assertTrue(testUtterance("shafer look up",
//        "INSTRUCT(brad,shafer,look(up),{})",
//        "brad",
//        "self"
//    ));

        assertTrue(testUtterance("when you do a squat make sure you nod after you stand up",
                "INSTRUCT(brad,dempster,modifyAction(dempster,doasquat(dempster),insert(nod(dempster)),after(standUp(dempster))),{})",
                "brad",
                "self"
        ));

//
//    testUtterance("add success effect holding the object to pickup the object");
//    //did(self ,modifyAction(did(self, pickUp(object)), insert(holding(object)), effect(success)))
//    semantics.add("INSTRUCT(brad,andy,modifyAction(pickup(VAR0),insert(holding(VAR1)),effect(success)),{object(VAR0),object(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})");
//
//    testUtterance("remove success effect holding the object to pickup the object");
//    //did(self ,modifyAction(did(self, pickUp(object)), insert(holding(object)), effect(success)))
//    semantics.add("INSTRUCT(brad,andy,modifyAction(pickup(VAR0),remove(holding(VAR1)),effect(success)),{object(VAR0),object(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})");

//     testUtterance("add pre condition sees the object to pickup the object");
//    semantics.add("INSTRUCT(brad,andy,modifyAction(pickup(VAR0),insert(sees(VAR1)),condition(pre))),{})");

//    testUtterance("turn right ninety degrees");
//    semantics.add("INSTRUCT(brad,andy,turn(right,ninety),{})");


        assertTrue(testUtterance("start over andy",
                "INSTRUCT(brad,andy,startOver(andy),{})",
                "brad",
                "self"
        ));

//    testUtterance("when you dance replace turn right ninety degrees with turn left ninety degrees after lower your arms");
//    semantics.add("INSTRUCT(brad,andy,modifyAction(dance(),replace(with(turn(left,ninety),turn(right,ninety))),after(lower(arms))),{})");

        assertTrue(testUtterance("when you pass me the knife replace move the knife forward with move the knife toward me after move the knife up",
                "INSTRUCT(brad,andy,modifyAction(pass(brad,VAR0),replace(with(moveObject(VAR3,toward,brad),moveObject(VAR2,forward))),after(moveObject(VAR1,up))),{knife(VAR0),knife(VAR1),knife(VAR2),knife(VAR3),DEFINITE(VAR0),DEFINITE(VAR1),DEFINITE(VAR2),DEFINITE(VAR3)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("when you pass me the object replace move the object forward with move the object toward me after move the object up",
                "INSTRUCT(brad,andy,modifyAction(pass(brad,VAR0),replace(with(moveObject(VAR3,toward,brad),moveObject(VAR2,forward))),after(moveObject(VAR1,up))),{object(VAR0),object(VAR1),object(VAR2),object(VAR3),DEFINITE(VAR0),DEFINITE(VAR1),DEFINITE(VAR2),DEFINITE(VAR3)})",
                "brad",
                "self"
        ));

//    //when you pass me the knife after move the knife up replace move the knife forward with move the knife toward me
//    testUtterance("when you pass me the object after move the object up replace move the object forward with move the object toward me ");
//    semantics.add("INSTRUCT(brad,andy,modifyAction(pass(brad,VAR0),replace(with(moveObject(VAR3,toward,brad),moveObject(VAR2,forward))),after(moveObject(VAR1,up))),{object(VAR0),object(VAR1),object(VAR2),object(VAR3),DEFINITE(VAR0),DEFINITE(VAR1),DEFINITE(VAR2),DEFINITE(VAR3)})");


        //when you pass me the knife replace move the knife forward with move the knife toward me
        assertTrue(testUtterance("when you pass me the object replace move the object forward with move the object toward me ",
                "INSTRUCT(brad,andy,modifyAction(pass(brad,VAR0),replace(with(moveObject(VAR2,toward,brad),moveObject(VAR1,forward))),none()),{object(VAR0),object(VAR1),object(VAR2),DEFINITE(VAR0),DEFINITE(VAR1),DEFINITE(VAR2)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("show me the object",
                "INSTRUCT(brad,andy,show(brad,VAR0),{object(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("show me the object is like pass me the object but remove release the object",
                "INSTRUCT(brad,andy,modifyAction(like(show(brad,VAR0),pass(brad,VAR1)),remove(releaseObject(VAR2)),none()),{object(VAR0),object(VAR1),object(VAR2),DEFINITE(VAR0),DEFINITE(VAR1),DEFINITE(VAR2)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("show me the object is like pass me the object remove release the object",
                "INSTRUCT(brad,andy,modifyAction(like(show(brad,VAR0),pass(brad,VAR1)),remove(releaseObject(VAR2)),none()),{object(VAR0),object(VAR1),object(VAR2),DEFINITE(VAR0),DEFINITE(VAR1),DEFINITE(VAR2)})",
                "brad",
                "self"
        ));

        //testUtterance("show me the object is like pass me the object remove release the object");
        assertTrue(testUtterance("show me the object is like pass me the object without release the object",
                "INSTRUCT(brad,andy,modifyAction(like(show(brad,VAR0),pass(brad,VAR1)),remove(releaseObject(VAR2)),none()),{object(VAR0),object(VAR1),object(VAR2),DEFINITE(VAR0),DEFINITE(VAR1),DEFINITE(VAR2)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("hand me the object",
                "INSTRUCT(brad,andy,give(brad,VAR0),{object(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));
//
//    //When you successfully pickup the object you are holding the object
//    //testUtterance("add success effect holding the object to pickup the object");
//    //did(self ,modifyAction(did(self, pickUp(object)), insert(holding(object)), effect(success)))
//    testUtterance("after you pickup the object successfully you are holding the object");
//    semantics.add("INSTRUCT(brad,andy,modifyAction(pickup(VAR0),insert(holding(andy,VAR1)),effect(success)),{object(VAR0),object(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})");

//    testUtterance("after you fail to pickup the object successfully you are holding the object");
//    semantics.add("INSTRUCT(brad,andy,modifyAction(pickup(VAR0),insert(holding(andy,VAR1)),effect(failure)),{object(VAR0),object(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})");

        //Todo: This was commented in before and is the only thing that doesn't work - commenting it out for now
//    assertTrue(testUtterance("add success effect you are holding the object to you pickup the object",
//        "INSTRUCT(brad,andy,modifyAction(pickup(VAR0),insert(holding(andy,VAR1)),effect(success)),{object(VAR0),object(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
//        "brad",
//        "self"
//    ));

        assertTrue(testUtterance("do you know what time it is",
                "QUESTION(brad,andy,know(andy,currentTime),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("what time is it",
                "QUESTION(brad,andy,timeIs(WHAT),{})",
                "brad",
                "self"
        ));


//    assertTrue(testUtterances(text, semantics));
*/
    }

    @Test
    public void scorpioInterruptionTest() {
        log.info("InterruptionTest");
//        component.addDictionary("multiRobotMedkit.dict");
        component.addDictionary("poc4.dict");


        //Part 1

        assertTrue(testUtterance("do you see the object",
                "QUESTION(brad,self,see(self,VAR0:physobj),{object(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("pickup the object",
                "INSTRUCT(brad,self,pickup(self,VAR0:physobj),{object(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("pick up the object",
                "INSTRUCT(brad,self,pickup(self,VAR0:physobj),{object(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("release the object",
                "INSTRUCT(brad,self,release(self,VAR0:physobj),{object(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("put down the object",
                "INSTRUCT(brad,self,putDown(self,VAR0:physobj),{object(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("this object is a box",
                "STATEMENT(brad,self,instanceOf(VAR0:physobj,box:property),{object(VAR0:physobj),REFACTIVATED(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("this object is a bowl",
                "STATEMENT(brad,self,instanceOf(VAR0:physobj,bowl:property),{object(VAR0:physobj),REFACTIVATED(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("this object is a cup",
                "STATEMENT(brad,self,instanceOf(VAR0:physobj,cup:property),{object(VAR0:physobj),REFACTIVATED(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("do you see the bowl",
                "QUESTION(brad,self,see(self,VAR0:physobj),{bowl(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("do you see the box",
                "QUESTION(brad,self,see(self,VAR0:physobj),{box(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("do you see the red cup",
                "QUESTION(brad,self,see(self,VAR0:physobj),{cup(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("do you see the white cup",
                "QUESTION(brad,self,see(self,VAR0:physobj),{cup(VAR0:physobj),white(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        //TODO:brad: do we want property of here?
        assertTrue(testUtterance("there are marbles in the white cup",
                "STATEMENT(brad,self,in(VAR0:physobj,marbles),{cup(VAR0:physobj),white(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("there are nuts in the red cup",
                "STATEMENT(brad,self,in(VAR0:physobj,nuts),{cup(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("I want marbles in the bowl",
                "STATEMENT(brad,self,want(brad,in(VAR0:physobj,marbles)),{bowl(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("Wait",
                "INSTRUCT(brad,self,cancelGoal(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("I want nuts in the box",
                "STATEMENT(brad,self,want(brad,in(VAR0:physobj,nuts)),{box(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

//    assertTrue(testUtterance("Wait I want nuts in the box",
//      Arrays.asList("INSTRUCT(brad,self,cancelGoal(),{})","STATEMENT(brad,self,want(brad,in(VAR0:physobj,nuts)),{box(VAR0:physobj),DEFINITE(VAR0:physobj)})"),
//      "brad",
//      "self"
//    ));

        assertTrue(testUtterance("Wait I want nuts in the box first",
                "INSTRUCT(brad,self,supersedeCurrentGoal(self,want(brad,in(VAR0:physobj,nuts))),{box(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

//    assertTrue(testUtterance("resume your interrupted goal",
//            Arrays.asList("INSTRUCT(brad,self,resumeInterruptedGoal(),{})"),
//            "brad",
//            "self"
//    ));

        assertTrue(testUtterance("Stop I want nuts in the box instead",
                "INSTRUCT(brad,self,supersedeAndUndo(self,want(brad,in(VAR0:physobj,nuts))),{box(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

//    assertTrue(testUtterance("describe your plan",
//      "INSTRUCT(brad,self,describe(how(to(planned()))),{})",
//      "brad",
//      "self"
//    ));
    }

    @Test
    public void dolphinDemoTest() {
        log.info("dolphinDemoTest");
        component.addDictionary("temi.dict");

        //wrappers for info set by Temi
        List<String> locations;
        List<String> videos;

//      //Video 1
        assertTrue(testUtterance("Hello Temi",
                "GREETING(brad,self,hello,{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Please follow me",
                "INSTRUCT(brad,self,followMeBlocking(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Save this location as waiting room",
                "INSTRUCT(brad,self,saveLocation(self,\"waiting room\"),{})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Save this location as reception",
                "INSTRUCT(brad,self,saveLocation(self,reception),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("waiting room", "LOC", "\"waiting room\"", "DEFINITE");
        component.injectDictionaryEntry("reception", "LOC", "reception", "DEFINITE");

        assertTrue(testUtterance("go to location the waiting room",
//        "INSTRUCT(brad,self,goToLocation(waiting-room),{})",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{\"waiting room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("come to the waiting room",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{\"waiting room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //Video 2

//        assertTrue(testUtterance("the greeting destination is the reception",
//                "INSTRUCT(brad,self,setGreetDest(self,VAR0),{reception(VAR0),DEFINITE(VAR0)})",
//                "brad",
//                "self"
//        ));
//
//        assertTrue(testUtterance("greet patients in the the waiting room",
//                "INSTRUCT(brad,self,greet(self,VAR0),{\"waiting room\"(VAR0),DEFINITE(VAR0)})",
//                "brad",
//                "self"
//        ));


        component.injectDictionaryEntry("treatment chair three", "LOC", "\"treatment chair three\"", "DEFINITE");

        assertTrue(testUtterance("Escort the patient to the treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR0,patient),{\"treatment chair three\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex to the treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR0,alex),{\"treatment chair three\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex from the waiting room to the treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,alex,VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));



        assertTrue(testUtterance("display test",
                "INSTRUCT(brad,self,display(self,test),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("treatment chair three", "LOC", "\"treatment chair three\"", "DEFINITE");
        component.injectDictionaryEntry("chair one", "LOC", "\"chair one\"", "DEFINITE");
        component.injectDictionaryEntry("chair four", "LOC", "\"chair four\"", "DEFINITE");

        assertTrue(testUtterance("Escort",
                "UNKNOWN(brad,self,error(self,doNotKnowWhat(self,escort,means)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort patient",
                "UNKNOWN(brad,self,error(self,doNotKnowWhat(self,escort,means)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex",
                "UNKNOWN(brad,self,error(self,doNotKnowWhat(self,escort,means)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort from chair one",
                "UNKNOWN(brad,self,[Escort, from, chair, one],{})", // i.e., does not parse
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort the patient from chair one",
                "UNKNOWN(brad,self,[Escort, the, patient, from, chair, one],{})", // i.e., does not parse
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex from chair one",
                "UNKNOWN(brad,self,[Escort, Alex, from, chair, one],{})", // i.e., does not parse
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort to chair one",
                "INSTRUCT(brad,self,escort(self,VAR0,patient),{\"chair one\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort the patient to chair one",
                "INSTRUCT(brad,self,escort(self,VAR0,patient),{\"chair one\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex to chair one",
                "INSTRUCT(brad,self,escort(self,VAR0,alex),{\"chair one\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort from chair one to chair four",
                "INSTRUCT(brad,self,escort(self,VAR1,patient,VAR0),{\"chair one\"(VAR0),\"chair four\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort the patient from chair one to chair four",
                "INSTRUCT(brad,self,escort(self,VAR1,patient,VAR0),{\"chair one\"(VAR0),\"chair four\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex from chair one to chair four",
                "INSTRUCT(brad,self,escort(self,VAR1,alex,VAR0),{\"chair one\"(VAR0),\"chair four\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort the patient to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR0,patient),{\"treatment chair three\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR0,alex),{\"treatment chair three\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex from the waiting room to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,alex,VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort waiting room",
                "UNKNOWN(brad,self,error(self,doNotKnowWhat(self,escort,means)),{})",
                "brad",
                "self"
        ));

//    locations = new ArrayList<>();
//    locations.add("lab");
//    component.addLocations(locations);
        component.injectDictionaryEntry("lab", "LOC", "lab", "DEFINITE");

        assertTrue(testUtterance("Fetch me Alex's retainer from the lab",
                "INSTRUCT(brad,self,fetch(self,\"alex's retainer\",VAR0),{lab(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("give a tour",
                "INSTRUCT(brad,self,tour(self),{})",
                "brad",
                "self"
        ));

        //Video 3
        assertTrue(testUtterance("Say hello and welcome to the orthodontist",
                "INSTRUCT(brad,self,generateResponseFromString(self,\"hello and welcome to the orthodontist\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Display message always remember to floss",
                "INSTRUCT(brad,self,display(self,\"always remember to floss\"),{})",
                "brad",
                "self"
        ));

//    videos= new ArrayList<>();
//    videos.add("flossing");
//    component.addVideoNames(videos);
//
//    assertTrue(testUtterance("Play video flossing",
//            "INSTRUCT(brad,self,playVideo(flossing),{})",
//            "brad",
//            "self"
//    ));

        assertTrue(testUtterance("Save this location as waiting room",
                "INSTRUCT(brad,self,saveLocation(self,\"waiting room\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("follow me",
                "INSTRUCT(brad,self,followMeBlocking(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Save this location as reception",
                "INSTRUCT(brad,self,saveLocation(self,reception),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to the waiting room",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{\"waiting room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //Video 4

        assertTrue(testUtterance("fetch me a goody bag",
                "INSTRUCT(brad,self,fetch(self,\"a goody bag\"),{})",
                "brad",
                "self"
        ));
// TODO:brad: start with a wildcard?
//      //TODO:brad: plurals
//      assertTrue(testUtterance("goody bags are stored in the supply room",
//        "INSTRUCT(brad,self,setStorageArea(goody-bag,supply-room),{})",
//        "brad",
//        "self"
//      ));

//    locations = new ArrayList<>();
//    locations.add("supply room");
//    component.addLocations(locations);
        component.injectDictionaryEntry("supply room", "LOC", "\"supply room\"", "DEFINITE");

        assertTrue(testUtterance("supplies are stored in the supply room",
                "INSTRUCT(brad,self,setStorageArea(self,VAR0),{\"supply room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("fetch me a goody bag from the supply room",
                "INSTRUCT(brad,self,fetch(self,\"a goody bag\",VAR0),{\"supply room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("I will teach you how to give a tour",
                "STATEMENT(brad,self,will(brad,teach(self,to(tour(self)))),{})",
                "brad",
                "self"
        ));

//    locations = new ArrayList<>();
//    locations.add("treatment room");
//    component.addLocations(locations);
        component.injectDictionaryEntry("treatment room", "LOC", "\"treatment room\"", "DEFINITE");

        assertTrue(testUtterance("first go to the treatment room",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{\"treatment room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("then display message this is the treatment room",
                "INSTRUCT(brad,self,display(self,\"this is the treatment room\"),{})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("then say this is the treatment room where we help straighten your teeth",
                "INSTRUCT(brad,self,generateResponseFromString(self,\"this is the treatment room where we help straighten your teeth\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("That is how you give a tour",
                "STATEMENT(brad,self,endTeaching(self,tour(self)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("delete location the waiting room",
                "INSTRUCT(brad,self,deleteLocation(self,VAR0),{\"waiting room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to location the waiting room then say this is a test",
                "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,\"this is a test\")),{\"waiting room\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

//    videos= new ArrayList<>();
//    videos.add("brushing with braces");
//    component.addVideoNames(videos);
//
//    assertTrue(testUtterance("go to location the waiting room then play video brushing with braces",
//            "INSTRUCT(brad,self,then(goToLocation(self,VAR0,true),playVideo(self,brushing with braces)),{waiting room(VAR0),DEFINITE(VAR0)})",
//            "brad",
//            "self"
//    ));

        //TODO:brad: the morpheme wildcard changes break this. do we care?
//      assertTrue(testUtterance("take this to waiting room",
//        "INSTRUCT(brad,self,goToLocation(self,VAR0),{waiting room(VAR0),DEFINITE(VAR0)})",
//        "brad",
//        "self"
//      ));


        //Video 4 extra

//      assertTrue(testUtterance("I will teach you how to do the flossing demo for a patient in a treatment room",
//        "",
//        "brad",
//        "self"
//      ));
//      assertTrue(testUtterance("First go to the room",
//        "",
//        "brad",
//        "self"
//      ));
//      assertTrue(testUtterance("Then say hello patient",
//        "",
//        "brad",
//        "self"
//      ));
//      assertTrue(testUtterance("Then say I am going to show you a video about how to properly floss",
//        "",
//        "brad",
//        "self"
//      ));
//      assertTrue(testUtterance("Then play video flossing",
//        "",
//        "brad",
//        "self"
//      ));
//
//      assertTrue(testUtterance("That is how you do the flossing demo for a patient in a treatment room",
//        "",
//        "brad",
//        "self"
//      ));
//      assertTrue(testUtterance("do the flossing demo for a patient in a treatment room",
//        "",
//        "brad",
//        "self"
//      ));


        //Revised video contents

//      -Standing next to the front desk and greeting the patient, the front desk then says "Temi, take "Brad" to chair 2. and Temi says please follow me and takes Brad to Chair 1 and says, here is your patient. or Brad.
//        -At the chair, the assistant calls Temi from a click on the screen, Temi arrives, and the assistant says, please take these instruments to sterile.
//      -The assistant calls Temi to the chair, or sends Temi to a chair with instructions to show how to turn a PE, or brush etc..
//      -The assistant says go to supply and bring me a ........


//      escort NAME from STARTLOCation to DESTINATION"

        //From emprical data:


        assertTrue(testUtterance("save this location as chair 2",
                "INSTRUCT(brad,self,saveLocation(self,\"chair 2\"),{})",
                "brad",
                "self"
        ));

        //TODO:brad: should we pass in location as the semantic type here?
        component.getDictionary().generateLocationRules("chair 2", "",false);

//brad: keeping this as a reference about how the homophone stuff works
//      assertTrue(testUtterance("save this location as chair to",
//        "INSTRUCT(brad,self,saveLocation(chair to),{})",
//        "brad",
//        "self"
//      ));
//
//      locations = new ArrayList<>();
//      locations.add("chair to");
//      component.addLocations(locations,null);

      assertTrue(testUtterance("escort kate to chair two ",
        "INSTRUCT(brad,self,escort(self,VAR0,kate),{\"chair 2\"(VAR0),DEFINITE(VAR0)})",
        "brad",
        "self"
      ));


      assertTrue(testUtterance("escort kate to chair to",
        "INSTRUCT(brad,self,escort(self,VAR0,kate),{\"chair 2\"(VAR0),DEFINITE(VAR0)})",
        "brad",
        "self"
      ));

        //brad: keeping this as a reference about how the homophone stuff works
//      assertTrue(testUtterance("escort kate 2 chair 2",
//        "INSTRUCT(brad,self,escort(chair two,kate),{})",
//        "brad",
//        "self"
//      ));

//    locations = new ArrayList<>();
//    locations.add("chair three");
//    component.addLocations(locations);

        component.injectDictionaryEntry("chair three", "LOC", "\"chair three\"", "DEFINITE");

//brad: keeping this as a reference about how the homophone stuff works
//      assertTrue(testUtterance("escort kate 2 chair 3",
//        "INSTRUCT(brad,self,escort(chair three,kate),{})",
//        "brad",
//        "self"
//      ));

        assertTrue(testUtterance("Escort Alex from the chair to to the waiting room",
                "INSTRUCT(brad,self,escort(self,VAR1,alex,VAR0),{\"chair 2\"(VAR0),\"waiting room\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort Alex from the reception to the waiting room",
                "INSTRUCT(brad,self,escort(self,VAR1,alex,VAR0),{reception(VAR0),\"waiting room\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to the chair two",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{\"chair 2\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to the chair to",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{\"chair 2\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort the patient from the reception to the waiting room",
                "INSTRUCT(brad,self,escort(self,VAR1,patient,VAR0),{reception(VAR0),\"waiting room\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

//      take these two sterile
//    locations = new ArrayList<>();
//    locations.add("sterile");
//    component.addLocations(locations);

        component.injectDictionaryEntry("sterile", "LOC", "sterile", "DEFINITE");

        //TODO:brad: the morpheme wildcard changes break this. do we care? the take person vs take object distinction may be harder, i think we might want to for this case and not escort though
//      assertTrue(testUtterance("take these two sterile",
//        "INSTRUCT(brad,self,goToLocation(sterile),{})",
//        "brad",
//        "self"
//      ));

//      assertTrue(testUtterance("play video",
//        "INSTRUCT(brad,self,goToLocation(sterile),{})",
//        "brad",
//        "self"
//      ));

        //brad: keeping this as a reference about how the homophone stuff works
//      locations = new ArrayList<>();
//      locations.add("room four");
//      component.addLocations(locations);
//      assertTrue(testUtterance("Escort Jordan II room for",
//        "INSTRUCT(brad,self,escort(room four,jordan),{})",
//        "brad",
//        "self"
//      ));

        //TODO:brad: this still isn't working
        assertTrue(testUtterance("display",
                "UNKNOWN(brad,self,[display],{})", // i.e., does not parse
                "brad",
                "self"
        ));

        assertTrue(testUtterance("display message",
                "UNKNOWN(brad,self,[display, message],{})", // i.e., does not parse
                "brad",
                "self"
        ));

        assertTrue(testUtterance("remove goal from queue 1626190414059",
                "INSTRUCT(brad,self,cancelGoal(self,1626190414059),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort will from the waiting room to the treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,will,VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("save this location as will",
                "INSTRUCT(brad,self,saveLocation(self,will),{})",
                "brad",
                "self"
        ));
//    locations = new ArrayList<>();
//    locations.add("will");
//    component.addLocations(locations);

        component.injectDictionaryEntry("will", "LOC", "will", "DEFINITE");

        assertTrue(testUtterance("go to will",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{will(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to location will",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{will(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort brad and will from the waiting room to  treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,\"brad and will\",VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        //TODO:brad we also need to find the relevant changes for thse
        assertTrue(testUtterance("Escort from the waiting room to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,patient,VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort  from the waiting room to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,patient,VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort    TEST  from the waiting room to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,test,VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("escort test from the waiting room",
                "UNKNOWN(brad,self,[escort, test, from, the, waiting, room],{})", // i.e., does not parse
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Fetch a goody bag   from the lab",
                "INSTRUCT(brad,self,fetch(self,\"a goody bag\",VAR0),{lab(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Fetch     food   from the lab",
                "INSTRUCT(brad,self,fetch(self,food,VAR0),{lab(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

//        assertTrue(testUtterance("Get me a goody bag and Alex's retainer from the lab",
//                "INSTRUCT(brad,self,fetch(self,\"a goody bag and alex's retainer\",VAR0),{lab(VAR0),DEFINITE(VAR0)})",
//                "brad",
//                "self"
//        ));

      assertTrue(testUtterance("Escort DR. BRAD and DR. BRAD  from the waiting room to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,\"dr. brad and dr. brad\",VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

      assertTrue(testUtterance("Escort brad will from the waiting room to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,\"brad will\",VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("save this location as OFFICE",
                "INSTRUCT(brad,self,saveLocation(self,office),{})",
                "brad",
                "self"
        ));
//    locations = new ArrayList<>();
//    locations.add("office");
//    component.addLocations(locations);

        component.injectDictionaryEntry("office", "LOC", "office", "DEFINITE");

        assertTrue(testUtterance("go to location OFFICE",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{office(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("delete location OFFICE",
                "INSTRUCT(brad,self,deleteLocation(self,VAR0),{office(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to the office then say hello",
                "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,hello)),{office(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

      assertTrue(testUtterance("go to the office then say Escort me please",
              "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,\"escort me please\")),{office(VAR0),DEFINITE(VAR0)})",
              "brad",
              "self"
      ));

//      //TODO:brad: catch the case where the loc isn't known, and it just make sit a say text
//      assertTrue(testUtterance("go to temp and say get me food from the pantry",
//        "UNKNOWN(brad,self,[go, to, temp, and, say, get, me, food, from, the, pantry],{})", // i.e., does not parse
//              "brad",
//              "self"
//      ));

//    locations = new ArrayList<>();
//    locations.add("temp");
//    component.addLocations(locations);

        component.injectDictionaryEntry("temp", "LOC", "temp", "DEFINITE");

       assertTrue(testUtterance("go to temp and say get me food from the pantry",
               "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,\"get me food from the pantry\")),{temp(VAR0),DEFINITE(VAR0)})",
               "brad",
               "self"
       ));

      assertTrue(testUtterance("go to temp and say I want to be able to say this sentence",
               "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,\"i want to be able to say this sentence\")),{temp(VAR0),DEFINITE(VAR0)})",
               "brad",
               "self"
       ));

      assertTrue(testUtterance("go to temp and say I know how to fetch and escort and do so many things I want to ",
               "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,\"i know how to fetch and escort and do so many things i want to\")),{temp(VAR0),DEFINITE(VAR0)})",
               "brad",
               "self"
       ));

      assertTrue(testUtterance("go to temp and say I know how to do so many things ",
               "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,\"i know how to do so many things\")),{temp(VAR0),DEFINITE(VAR0)})",
               "brad",
               "self"
       ));


      assertTrue(testUtterance("say this is a test, and so is this",
              "INSTRUCT(brad,self,generateResponseFromString(self,\"this is a test, and so is this\"),{})",
              "brad",
              "self"
      ));

        assertTrue(testUtterance("say this is a test, and so is this",
                "INSTRUCT(brad,self,generateResponseFromString(self,\"this is a test, and so is this\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to the temp and say I know how to do so many things ",
                "INSTRUCT(brad,self,then(self,goToLocation(self,VAR0,true),generateResponseFromString(self,\"i know how to do so many things\")),{temp(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

//        assertTrue(testUtterance("say !@#$%^&*():'\"/?.,;`~{[}] and more",
//                "INSTRUCT(brad,self,sayText(self,!@#$%^&*():'\"/?.,;`~{[}] and more),{})",
//                "brad",
//                "self"
//        ));
                assertTrue(testUtterance("say !@#$%^&*(:'\"/?.;`~{[}] and more",
                "INSTRUCT(brad,self,generateResponseFromString(self,\"!@#$%^&*(:'\"/?.;`~{[}] and more\"),{})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("",
                "UNKNOWN(brad,self,[],{})", // i.e., does not parse
                "brad",
                "self"
        ));

        assertTrue(testUtterance("set volume to one",
                "INSTRUCT(brad,self,setVolume(self,one),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("set volume two",
                "INSTRUCT(brad,self,setVolume(self,two),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("mute",
                "INSTRUCT(brad,self,setVolume(self,zero),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("mute volume",
                "INSTRUCT(brad,self,setVolume(self,zero),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("unmute",
                "INSTRUCT(brad,self,setVolume(self,three),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("unmute volume",
                "INSTRUCT(brad,self,setVolume(self,three),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("sendmap",
                "INSTRUCT(brad,self,sendMap(self,true),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("sendlogs",
                "INSTRUCT(brad,self,sendLogs(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("say ?ACTOR ? who's that?",
                "INSTRUCT(brad,self,generateResponseFromString(self,\"?actor ? who's that?\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("say I am capitalized",
                "INSTRUCT(brad,self,generateResponseFromString(self,\"i am capitalized\"),{})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("Display qr code https://app.znorobotics.com?#_",
                "INSTRUCT(brad,self,displayQRCode(self,\"https://app.znorobotics.com?#_\",\"\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Display qr code https://app.znorobotics.com?#_ with message welcome to zno robotics!",
                "INSTRUCT(brad,self,displayQRCode(self,\"https://app.znorobotics.com?#_\",\"welcome to zno robotics!\"),{})",
                "brad",
                "self"
        ));
//        assertTrue(testUtterance("Display qr code https://app.znorobotics.com?#_ with message H0w wOu1_d I even,.. . say t$is",
//                "INSTRUCT(brad,self,displayQRCode(self,\"https://app.znorobotics.com?#_\",\"H0w wOu1_d I even,.. . say t$is\"),{})",
//                "brad",
//                "self"
//        ));

        component.injectDictionaryEntry("dr. eric's office", "LOC", "\"dr. eric's office\"", "DEFINITE");


        //TODO: update to pass/fail correctly
        assertTrue(testUtterance("Escort Alex,s test from the waiting room to treatment chair three",
                "INSTRUCT(brad,self,escort(self,VAR1,\"alex,s test\",VAR0),{\"waiting room\"(VAR0),\"treatment chair three\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort pro.,ble:matictext! from chair one to chair one",
                "INSTRUCT(brad,self,escort(self,VAR1,\"pro.,ble:matictext!\",VAR0),{\"chair one\"(VAR0),\"chair one\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("fetch test1.5 from chair one",
                "INSTRUCT(brad,self,fetch(self,\"test1.5\",VAR0),{\"chair one\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("fetch test 1.5 from chair one",
                "INSTRUCT(brad,self,fetch(self,\"test 1.5\",VAR0),{\"chair one\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("fetch test one point five from chair one",
                "INSTRUCT(brad,self,fetch(self,\"test one point five\",VAR0),{\"chair one\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("fetch t3st from chair one",
                "INSTRUCT(brad,self,fetch(self,t3st,VAR0),{\"chair one\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Escort t3st from chair one to chair four",
                "INSTRUCT(brad,self,escort(self,VAR1,t3st,VAR0),{\"chair one\"(VAR0),\"chair four\"(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("go to location dr. eric's office",
//        "INSTRUCT(brad,self,goToLocation(waiting-room),{})",
                "INSTRUCT(brad,self,goToLocation(self,VAR0,true),{\"dr. eric's office\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Save this location as !test",
                "INSTRUCT(brad,self,saveLocation(self,\"!test\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Save this location as ?test",
                "INSTRUCT(brad,self,saveLocation(self,\"?test\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Save this location as Test",
                "INSTRUCT(brad,self,saveLocation(self,test),{})",
                "brad",
                "self"
        ));
    }

    @Test
    public void pourTest() {
        log.info("pourTest");
        component.addDictionary("multiRobotMedkit.dict");
        component.addDictionary("poc4.dict");

        assertTrue(testUtterance("look for containers",
                "INSTRUCT(brad,self,startPourSearch(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there are nuts in the red jar",
                "STATEMENT(brad,self,in(VAR0:physobj,nuts),{jar(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("pick up the red jar",
                "INSTRUCT(brad,self,pickup(self,VAR0:physobj),{jar(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("I will teach you how to pour nuts from the red jar into the purple jar",
                "STATEMENT(brad,self,will(teach(brad,self,to(pour(self,nuts,VAR0:physobj,VAR1:physobj)))),{jar(VAR0:physobj),red(VAR0:physobj),jar(VAR1:physobj),purple(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("first pick up the red jar",
                "INSTRUCT(brad,self,pickup(self,VAR0:physobj),{jar(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Then get above the purple jar",
                "INSTRUCT(brad,self,sti(self,above(VAR0:physobj)),{jar(VAR0:physobj),purple(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Then tilt the red jar",
                "INSTRUCT(brad,self,tilt(self,VAR0:physobj),{jar(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("That is how you pour nuts from the red jar into the purple jar",
                "STATEMENT(brad,self,endTeaching(self,pour(self,nuts,VAR0:physobj,VAR1:physobj)),{jar(VAR0:physobj),red(VAR0:physobj),jar(VAR1:physobj),purple(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Pour nuts from the red jar into the purple jar",
                "INSTRUCT(brad,self,pour(self,nuts,VAR0:physobj,VAR1:physobj),{jar(VAR0:physobj),red(VAR0:physobj),jar(VAR1:physobj),purple(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("When you pour nuts from the red jar into the purple jar make sure the red jar is open before you get above the purple jar",
                "INSTRUCT(brad,self,modifyAction(self,pour(self,nuts,VAR0:physobj,VAR1:physobj),insert(isopen(VAR2:physobj)),before(sti(self,above(VAR3:physobj)))),{jar(VAR0:physobj),red(VAR0:physobj),jar(VAR1:physobj),purple(VAR1:physobj),jar(VAR2:physobj),red(VAR2:physobj),jar(VAR3:physobj),purple(VAR3:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj),DEFINITE(VAR2:physobj),DEFINITE(VAR3:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Save this pose as human",
                "INSTRUCT(brad,self,recordEEPose(self,human),{})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("I will teach you how to open the jar",
                "STATEMENT(brad,self,will(teach(brad,self,to(open(VAR0:physobj)))),{jar(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        //TODO:brad: the ee pose syntax is used with the DIARC vision based actions which are used in the demo, but IDK how that works tih the Assista
        assertTrue(testUtterance("First go to pose human",
                "INSTRUCT(brad,self,goToEEPose(self,human),{})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Then say please open this",
                "INSTRUCT(brad,self,generateResponseFromString(self,\"please open this\"),{})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Then wait for acknowledgement",
                "INSTRUCT(brad,self,waitForAck(self),{})",
                "brad",
                "self"
        ));
//        assertTrue(testUtterance("add success effect the jar is open",
//                "INSTRUCT(brad,self,pickup(self,VAR0:physobj),{jar(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
//                "brad",
//                "self"
//        ));
        assertTrue(testUtterance("that is how you open the jar",
                "STATEMENT(brad,self,endTeaching(self,open(VAR0:physobj)),{jar(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));
//        assertTrue(testUtterance("add success effect there are nuts in the purple jar to pour nuts from the red jar into the purple jar",
//                "INSTRUCT(brad,self,pickup(self,VAR0:physobj),{jar(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
//                "brad",
//                "self"
//        ));
//        assertTrue(testUtterance("add success effect there are not nuts in the red jar to pour nuts from the red jar into the purple jar",
//                "INSTRUCT(brad,self,pickup(self,VAR0:physobj),{jar(VAR0:physobj),red(VAR0:physobj),DEFINITE(VAR0:physobj)})",
//                "brad",
//                "self"
//        ));
        assertTrue(testUtterance("the purple jar is open",
                "STATEMENT(brad,self,isopen(VAR0:physobj),{jar(VAR0:physobj),purple(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("I want nuts in the purple jar",
                "STATEMENT(brad,self,want(brad,in(VAR0:physobj,nuts)),{jar(VAR0:physobj),purple(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));
        assertTrue(testUtterance("Stop I want nuts in the purple jar instead",
                "INSTRUCT(brad,self,supersedeAndUndo(self,want(brad,in(VAR0:physobj,nuts))),{jar(VAR0:physobj),purple(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));
    }

    @Test
    public void multiRobotMedkitTest() {
        log.info("multiRobotMedkitTest");
        component.addDictionary("multiRobotMedkit.dict");
        component.addDictionary("poc4.dict");

//        Medkit/Package teaching:

        //TODO:brad: define new recipe standard medkit?

        // H:  "a standard medkit contains bandages, antiseptic, and painkillers"
        assertTrue(testUtterance("a standard medkit contains bandages antiseptic and painkillers",
                "STATEMENT(medic,self,recipe(standardMedkit,contains(bandages,antiseptic,painkillers)),{})",
                "medic",
                "self"
        ));

        //TODO: come up with the target predicate representation for this
        // H: "a standard medkit uses a medical caddy"

        // Loading teaching:

        //TODO:brad: ad functionality to parser tests to represent different listener info
        assertTrue(testUtterance("follow me",
                "INSTRUCT(medic,robotTwo,followMeBlocking(robotTwo),{})",
                "medic",
                "robotTwo"
        ));

//        H:save this location as loading area
        assertTrue(testUtterance("save this location as loading area",
                "INSTRUCT(medic,robotTwo,saveLocation(robotTwo,\"loading area\"),{})",
                "medic",
                "robotTwo"
        ));

        component.injectDictionaryEntry("loading area", "LOC", "\"loading area\"", "DEFINITE");

        //TODO:brad: should self be the actor here?
//        H: "Save this pose as drop off"
                assertTrue(testUtterance("save this pose as drop off",
                "INSTRUCT(medic,self,recordEEPose(self,\"drop off\"),{})",
                "medic",
                "self"
        ));

        component.injectDictionaryEntry("drop off", "POSE", "\"drop off\"", "DEFINITE");

       //TODO:brad: actual pose consultant? for UR 5
//        H(via tts on mobile app): "it is above loading area"
        assertTrue(testUtterance("it is above loading area",
                "INSTRUCT(medic,self,bindPose(self,VAR0,VAR1),{it(VAR0),\"loading area\"(VAR1),INFOCUS(VAR0),DEFINITE(VAR1)})",
                "medic",
                "self"
        ));

        // H: "Save this pose as "conveyor pick up"
        assertTrue(testUtterance("save this pose as conveyor pickup",
                "INSTRUCT(medic,self,recordEEPose(self,\"conveyor pickup\"),{})",
                "medic",
                "self"
        ));

//        H: "it is above conveyor end"
        assertTrue(testUtterance("it is above conveyor end",
                "INSTRUCT(medic,self,bindPose(self,VAR0,conveyor_end),{it(VAR0),INFOCUS(VAR0)})",
                "medic",
                "self"
        ));

        //        Search & rescue:
        assertTrue(testUtterance("follow me",
                "INSTRUCT(searcher,robotOne,followMeBlocking(robotOne),{})",
                "searcher",
                "robotOne"
        ));

        assertTrue(testUtterance("save this location as alpha",
                "INSTRUCT(searcher,robotOne,saveLocation(robotOne,alpha),{})",
                "searcher",
                "robotOne"
        ));

        component.injectDictionaryEntry("alpha", "LOC", "alpha", "DEFINITE");

        //TODO: is this going to mean anything symbolically, this is really a message to the other human, maybe we can have a send massage action?
        //  Searcher: robot one, there is a wounded person at robot one’s location
//        assertTrue(testUtterance("there is a wounded person at alpha",
//                "INSTRUCT(searcher,robotOne,at(wounded(person),VAR0),{alpha(VAR0),DEFINITE(VAR0)})",
//                "searcher",
//                "robotOne"
//        ));

        //TODO:brad: can we get a way with underscores here?
        assertTrue(testUtterance("send message there is a wounded person at alpha",
                "INSTRUCT(searcher,robotOne,sendMessage(robotOne,\"there is a wounded person at alpha\"),{})",
                "searcher",
                "robotOne"
        ));

        //TODO:brad: is this actually the fromat we want
        assertTrue(testUtterance("I need a standard medkit at alpha",
                "INSTRUCT(medic,self,deliver(self,standardMedkit,VAR0),{alpha(VAR0),DEFINITE(VAR0)})",
                "medic",
                "self"
        ));

        //TODO:brad:can we also get away with underscores here?
//        Medic(via web app):robot one, say, searcher, help is on the way, you can continue searching
        assertTrue(testUtterance("say searcher help is on the way you can continue searching",
                "INSTRUCT(medic,robotOne,generateResponseFromString(robotOne,\"searcher help is on the way you can continue searching\"),{})",
                "medic",
                "robotOne"
        ));

//        Robot one: "Searcher, help is on the way you  can continue searching"
//*medic goes to wounded person’s room"
//
//        Searcher: "follow me"
        assertTrue(testUtterance("follow me",
                "INSTRUCT(searcher,robotOne,followMeBlocking(robotOne),{})",
                "searcher",
                "robotOne"
        ));


        //TODO:brad: is bandages a ref?
        // Medic (to phone): I need another box of bandages at
        assertTrue(testUtterance("I need a standard medkit with only bandages at alpha",
                "INSTRUCT(medic,self,deliver(self,standardMedkit,only(bandages),VAR0),{alpha(VAR0),DEFINITE(VAR0)})",
                "medic",
                "self"
        ));

        assertTrue(testUtterance("bandages go in compartment one",
                "STATEMENT(medic,self,stores(compartmentOne,bandages),{})",
                "medic",
                "self"
        ));

        assertTrue(testUtterance("antiseptic goes in compartment two",
                "STATEMENT(medic,self,stores(compartmentTwo,antiseptic),{})",
                "medic",
                "self"
        ));

        assertTrue(testUtterance("painkillers go in compartment three",
                "STATEMENT(medic,self,stores(compartmentThree,painkillers),{})",
                "medic",
                "self"
        ));

        assertTrue(testUtterance("a standard medkit uses a medical caddy and contains bandages antiseptic and painkillers",
                "STATEMENT(medic,self,recipe(standardMedkit,medkit:property,contains(bandages,antiseptic,painkillers)),{})",
                "medic",
                "self"
        ));

    }

    @Test
    public void poc4Test() {
        log.info("poc4Test");
        component.addDictionary("poc4.dict");
        component.addDictionary("assemblyHomophones.dict");


        // VIDEO A.!
        assertTrue(testUtterance("Save this pose as conveyor",
                "INSTRUCT(brad,self,recordCameraPoseAsk(self,conveyor),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("conveyor", "RN", "conveyor", "VAR");
        component.injectDictionaryEntry("conveyor", "REF", "conveyor", "DEFINITE");

        assertTrue(testUtterance("20 millimeters",
                "REPLY(brad,self,val(20,mm),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Save this pose as work area",
                "INSTRUCT(brad,self,recordCameraPoseAsk(self,\"work area\"),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("work area", "RN", "\"work area\"", "VAR");
        component.injectDictionaryEntry("work area", "REF", "\"work area\"", "DEFINITE");

        assertTrue(testUtterance("5 mm",
                "REPLY(brad,self,val(5,mm),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Save this pose as screw feeder",
                "INSTRUCT(brad,self,recordCameraPoseAsk(self,\"screw feeder\"),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("screw feeder", "RN", "\"screw feeder\"", "VAR");
        component.injectDictionaryEntry("screw feeder", "REF", "\"screw feeder\"", "DEFINITE");

        assertTrue(testUtterance("10 mm",
                "REPLY(brad,self,val(10,mm),{})",
                "brad",
                "self"
        ));

        //TODO:brad: does the consultant work like this?
        component.injectDictionaryEntry("screw feeder", "POSE", "\"screw feeder\"", "VAR");


        //VIDEO A.2
        assertTrue(testUtterance("define new screw type M3",
                "INSTRUCT(brad,self,defineScrewType(self,m3),{})",
                "brad",
                "self"
        ));

        //what is its target torque in millinewton meters
        assertTrue(testUtterance("170 millinewton meters",
                "REPLY(brad,self,val(170,mNm),{})",
                "brad",
                "self"
        ));
        //what is its max torque in millinewton meters
        assertTrue(testUtterance("300 millinewton meters",
                "REPLY(brad,self,val(300,mNm),{})",
                "brad",
                "self"
        ));

        //what is its angle max in degrees
        assertTrue(testUtterance("6500 degrees",
                "REPLY(brad,self,val(6500,deg),{})",
                "brad",
                "self"
        ));

        //at which pose can I find m3 screws?

        assertTrue(testUtterance("pose screw feeder",
                "REPLY(brad,self,pose(VAR0),{\"screw feeder\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //VIDEO A3.i
        //TODO: maybe we can gnerify this?
        assertTrue(testUtterance("I will teach you how to screw an M3 screw into an M3 hole",
                "STATEMENT(brad,self,will(teach(brad,self,to(screwIn(self,m3,VAR0)))),{m3Hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("I will teach you how to screw a screw into a hole",
                "STATEMENT(brad,self,will(teach(brad,self,to(screwIn(self,screw,VAR0)))),{hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("first mount an m3 screw",
                "INSTRUCT(brad,self,mountScrew(self,m3),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("first verify that you can see the hole",
                "INSTRUCT(brad,self,perceiveEntity(self,VAR0),{hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("first mount the screw",
                "INSTRUCT(brad,self,mountScrew(self,screw),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then align with the m3 hole",
                "INSTRUCT(brad,self,alignWith(self,VAR0),{m3Hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then align with the hole",
                "INSTRUCT(brad,self,alignWith(self,VAR0),{hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then run the screwdriver job of the screw",
                "INSTRUCT(brad,self,runScrewdriverJob(self,screw),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("That is how you screw an M3 screw into an M3 hole",
                "STATEMENT(brad,self,endTeaching(self,screwIn(self,m3,VAR0)),{m3Hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("That is how you screw a screw into a hole",
                "STATEMENT(brad,self,endTeaching(self,screwIn(self,screw,VAR0)),{hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

//        //TODO:make this action say how many results it found? via tts?
        assertTrue(testUtterance("search for m3 holes",
                "INSTRUCT(brad,self,observeDescriptor(self,m3Hole,-1),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("screw an M3 screw into an M3 hole",
                "INSTRUCT(brad,self,screwIn(self,m3,VAR0),{m3Hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("screw-in M3 screw into an M3 hole",
                "INSTRUCT(brad,self,screwIn(self,m3,VAR0),{m3Hole(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //VIDEO A.3.ii
        assertTrue(testUtterance("update m3 screw target torque to 150 millinewton meters",
                "INSTRUCT(brad,self,updateParam(self,targetTorque(m3),val(150,mNm)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("update m3 screw max torque to 200 millinewton meters",
                "INSTRUCT(brad,self,updateParam(self,maxTorque(m3),val(200,mNm)),{})",
                "brad",
                "self"
        ));

        //VIDEO B.1
//        Human: Define new item NF32-SV circuit breaker
        assertTrue(testUtterance("define new item NF32-SV",
                "INSTRUCT(brad,self,defineItem(self,nf32sv),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("nf32-sv", "RN", "nf32sv", "VAR");
        component.injectDictionaryEntry("nf32-sv", "DESC", "nf32sv","");

        //which Cognex job is used to detect its hole
        assertTrue(testUtterance("job circuit breaker face",
                "REPLY(brad,self,job(cbDet),{})",
                "brad",
                "self"
        ));

//                Robot: "Okay. Does it have any other features?"
//        Human: It has 2 M3 screw holes on top.

        //VIDEO B.2


        assertTrue(testUtterance("I will teach you how to assemble an NF32-SV",
                "STATEMENT(brad,self,will(teach(brad,self,to(assemble(VAR0)))),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("that is how you assemble an NF32-SV",
                "STATEMENT(brad,self,endTeaching(self,assemble(VAR0)),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("first go to pose conveyor",
                "INSTRUCT(brad,self,goToCameraPose(self,VAR0),{conveyor(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //        Human: Then verify that you can see a.
        assertTrue(testUtterance("then verify that you can see an NF32-SV",
                "INSTRUCT(brad,self,perceiveEntity(self,VAR0),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));



//        Human: then get it on the work area
//        assertTrue(testUtterance("Then get it on the work area",
//                "INSTRUCT(brad,self,sti(self,on(VAR0,VAR1)),{it(VAR0),work area(VAR1),INFOCUS(VAR0),DEFINITE(VAR1)})",
//                "brad",
//                "self"
//        ));

        assertTrue(testUtterance("Then get it on the work area",
                "INSTRUCT(brad,self,getOn(self,VAR0,VAR1),{it(VAR0),\"work area\"(VAR1),INFOCUS(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("then search for 2 m3 holes",
                "INSTRUCT(brad,self,observeDescriptor(self,m3Hole,2),{})",
                "brad",
                "self"
        ));

//        Human: Then screw m3 screws into the two holes on top

        assertTrue(testUtterance("screw an M3 screw into the left M3 hole",
                "INSTRUCT(brad,self,screwIn(self,m3,VAR0),{m3Hole(VAR0),left(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("screw an M3 screw into the right M3 hole",
                "INSTRUCT(brad,self,screwIn(self,m3,VAR0),{m3Hole(VAR0),right(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Then get the NF32-SV on the conveyor",
                "INSTRUCT(brad,self,getOn(self,VAR0,VAR1),{nf32sv(VAR0),conveyor(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        //VIDEO B.3
        assertTrue(testUtterance("assemble a NF32-SV ",
                "INSTRUCT(brad,self,assemble(self,VAR0),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("assemble a NF-32SV ",
                "INSTRUCT(brad,self,assemble(self,VAR0),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //VIDE0 C.1

        assertTrue(testUtterance("generate robot code for assemble a NF32-SV",
                "INSTRUCT(brad,self,translateGoal(self,assemble(self,VAR0)),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //TODO:brad: character encoding issues?
//        assertTrue(testUtterance("m3 のネジでネジ締めしてください",
//                "INSTRUCT(brad,self,screwIn(self,m3,VAR0),{m3(VAR0),DEFINITE(VAR0)})",
//                "brad",
//                "self"
//        ));


        //small circut breakrker action modification
        assertTrue(testUtterance("define new item NV30-FAU",
                "INSTRUCT(brad,self,defineItem(self,nv30fau),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("nv30-fau", "RN", "nv30fau", "VAR");
        component.injectDictionaryEntry("nv30-fau", "DESC", "nv30fau","");

        //which Cognex job is used to detect its hole
        assertTrue(testUtterance("job n v det",
                "REPLY(brad,self,job(nvDet),{})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("assemble an NV30-FAU",
                "INSTRUCT(brad,self,assemble(self,VAR0),{nv30fau(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("assemble an NV30-FAU is like assemble an NF32-SV",
                "INSTRUCT(brad,self,modifyAssemble(self,assemble(self,VAR0),assemble(self,VAR1)),{nv30fau(VAR0),nf32sv(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        //replace screw in 3 m3 screws with screw in 5 m4 screws

        assertTrue(testUtterance("replace search for 2 m3 holes with search for 2 deep m3 holes",
                "REPLY(brad,self,mod(replace(observeDescriptor(self,deepM3Hole,2),observeDescriptor(self,m3Hole,2)),none()),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Replace screw an M3 screw into the left M3 hole with screw an M3 screw into the bottom deep M3 hole",
                "REPLY(brad,self,mod(replace(screwIn(self,m3,VAR1),screwIn(self,m3,VAR0)),none()),{m3Hole(VAR0),left(VAR0),deepM3Hole(VAR1),bottom(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Replace screw an M3 screw into the right M3 hole with screw an M3 screw into the top deep M3 hole",
                "REPLY(brad,self,mod(replace(screwIn(self,m3,VAR1),screwIn(self,m3,VAR0)),none()),{m3Hole(VAR0),right(VAR0),deepM3Hole(VAR1),top(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("that is all",
                "REPLY(brad,self,mod(none,none),{})",
                "brad",
                "self"
        ));

    }

  @Test
  public void tetherBotTest() {
    log.info("tetherBotTest");
    component.addDictionary("tetherbot.dict");

//    "I will teach you how to sterilize the office one"
//    This utterance indicates that the human is going to teach the TetherBot a new custom behavior
//
//"First go to the office"
//    This instructs TetherBot to navigate to the location which has been labeled "office"
//
//"Then connect to the dock"
//    This instructs TeterBot to connect to the nearest visible dock using the docking behavior described in section XXXX. It is assumed that a dock will be visible to the Tetherbot at this location. If it can not detect one it will provide an error explanation in natural language.
//
//"Then turn on payload"
//    This instructs the tetherbot to provide power to the outlet on it’s payload. Powering on the mounted sterilization device.
//
//"Then go to alpha"
//    TetherBot navigates to location "alpha".
//
//"Then wait for 10 seconds"
//    TetherBot waits for 10 seconds
//
//"Then go to beta"
//    TetherBot navigates to location "beta".
//
//"Then wait for 10 seconds"
//    TetherBot waits for 10 seconds
//
//"Then go to office"
//    TetherBot navigates to location "office".
//
//"Then undock"
//    TetherBot disconnects from the dock.
//
//"Then go to the hallway"
//    TetherBot navigates to location "hallway"
//
//"That is how you sterilize the office"
//
//"Sterilize the office."
//    The video XXXX shows the execution of this taught behavior.

    assertTrue(testUtterance("sterilize the office",
            "INSTRUCT(brad,self,cleanRoom(self,office),{})",
            "brad",
            "self"
    ));

    assertTrue(testUtterance("go to office",
            "INSTRUCT(brad,self,goTo(self,office,true),{})",
            "brad",
            "self"
    ));


  }

  @Test
  public void nasaDeepSpaceTest() {
      component.addDictionary("nasadeepspace.dict");
      component.addDictionary("poc4.dict");
      component.addDictionary("assemblyHomophones.dict");


      assertTrue(testUtterance("init",
              "INSTRUCT(brad,self,initGoalQueue(self),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("init goal queue problem solving",
              "INSTRUCT(brad,self,initGoalQueueProblemSolving(self),{})",
              "brad",
              "self"
      ));

      //eval case 1
      assertTrue(testUtterance("when is the last time experiment two was run",
              "INSTRUCT(brad,self,queryEventTime(self,experiment2(X)),{})",
              "brad",
              "self"
      ));
      assertTrue(testUtterance("was it successful",
              "INSTRUCT(brad,self,describeSuccess(self,VAR0:context),{it(VAR0:context),INFOCUS(VAR0:context)})",
              "brad",
              "self"
      ));
//      assertTrue(testUtterance("has work station 4 been repaired",
//              "INSTRUCT(brad,self,querySupport(self,repaired(X),{})", //todo: get binding
//              "brad",
//              "self"
//      ));
//      assertTrue(testUtterance("had workstation 4 been working would yesterday's run of experiment 5 succeeded?",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})", //todo
//              "brad",
//              "self"
//      ));
//
//      //Eval case 2 //todo: all
//      assertTrue(testUtterance(" how many experiments are currently scheduled?",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})",
//              "brad",
//              "self"
//      ));
//      assertTrue(testUtterance("How long would it take to execute experiment 5 right now?",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})",
//              "brad",
//              "self"
//      ));
//      assertTrue(testUtterance("yes",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})",
//              "brad",
//              "self"
//      ));
//
//      //eval case 3 //todo: all
//      assertTrue(testUtterance("What are you doing now?",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})",
//              "brad",
//              "self"
//      ));
//      assertTrue(testUtterance("how long will they take to complete?",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})",
//              "brad",
//              "self"
//      ));
//      assertTrue(testUtterance("how many experiments are in the queue?",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})",
//              "brad",
//              "self"
//      ));
//      assertTrue(testUtterance("add repair bay alpha left 3 to the top of the schedule",
//              "INSTRUCT(brad,self,initGoalQueue(self),{})",
//              "brad",
//              "self"
//      ));
//
      //eval case 4
      assertTrue(testUtterance("describe how to run experiment 8",
              "INSTRUCT(brad,self,describe(self,how(to(experiment8(self)))),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("add robot one run experiment 8 to the top of the schedule",
              "INSTRUCT(brad,self,submitGoalDialogue(self,experiment8(robotone:agent)),{})",
              "brad",
              "self"
      ));

      //eval case 5 //todo: all
      assertTrue(testUtterance("run experiment 7",
              "INSTRUCT(brad,self,experiment7(self),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("run experiment 9 is like run experiment 4",
              "INSTRUCT(brad,self,modifyLikeInteractive(self,experiment9(self),experiment4(self)),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("go to experiment station gamma right four",
              "INSTRUCT(brad,self,goToLocation(self,VAR0:location),{experimentStation(VAR0:location),in(VAR0:location,gamma:area),side(VAR0:location,right:direction),wingid(VAR0:location,four:id),DEFINITE(VAR0:location)})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("go to experiment station gamma right five",
              "INSTRUCT(brad,self,goToLocation(self,VAR0:location),{experimentStation(VAR0:location),in(VAR0:location,gamma:area),side(VAR0:location,right:direction),wingid(VAR0:location,five:id),DEFINITE(VAR0:location)})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("replace go to experiment station beta left four with go to experiment station gamma right five",
              "REPLY(brad,self,mod(replace(goToLocation(self,VAR1:location),goToLocation(self,VAR0:location)),none()),{experimentStation(VAR0:location),in(VAR0:location,beta:area),side(VAR0:location,left:direction),wingid(VAR0:location,four:id),experimentStation(VAR1:location),in(VAR1:location,gamma:area),side(VAR1:location,right:direction),wingid(VAR1:location,five:id),DEFINITE(VAR0:location),DEFINITE(VAR1:location)})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("replace check that tube temperature is 27 degrees with check that tube pressure is below 23 psi",
              "REPLY(brad,self,mod(replace(checkTubePressureIsBelow(self,psi(23)),checkTubeTemperatureIs(self,degrees(27))),none()),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("That is all",
              "REPLY(brad,self,mod(none),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("robot one run experiment 9",
              "INSTRUCT(brad,robotone:agent,experiment9(robotone:agent),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("robot one run experiment 8",
              "INSTRUCT(brad,robotone:agent,experiment8(robotone:agent),{})",
              "brad",
              "self"
      ));

      assertTrue(testUtterance("robot one add run experiment 8 to the top of the schedule",
              "INSTRUCT(brad,robotone:agent,submitGoalDialogue(robotone:agent,experiment8(robotone:agent)),{})",
              "brad",
              "robotone"
      ));


/*
      assertTrue(testUtterance("Add new notification",
              "INSTRUCT(brad,self,initGoalQueue(self),{})",
              "brad",
              "self"
      ));
      assertTrue(testUtterance("every time an experiment bay breaks",
              "INSTRUCT(brad,self,initGoalQueue(self),{})",
              "brad",
              "self"
      ));
      assertTrue(testUtterance("Tell me the duration of all currently running experiments",
              "INSTRUCT(brad,self,initGoalQueue(self),{})",
              "brad",
              "self"
      ));
      assertTrue(testUtterance("Tell me the number of scheduled experiments",
              "INSTRUCT(brad,self,initGoalQueue(self),{})",
              "brad",
              "self"
      ));
*/

//      H: When was the last time you ran experiment 5?
//              R: Yesterday at …
//      H: Was it successful?
//              R: No it failed because of a power outage
//      H: Was the power outage successfully resolved?
//      R: yes
//      H: had the power outage not occurred would yesterday's run of experiment 5 succeeded?

//      H: how many experiments are currently scheduled?
//      R: none
//      H: How long would it take to execute experiment 5 right now?
//      R: 15 minutes. Would you like me to execute it now?
//      H: yes
//              (alternatively if there were more experiments running the human could opt to add it to the back of the experiment queue)

//      R: experiment bay alpha left 3 has broken and is degrading experiment performance.
//              H: What are you doing now?
//              R: Experiments 5 and 7 are currently being executed.
//      H: how long will they take to complete?
//      R: experiment 5 will be completed in 3 minutes, and experiment 7 will be completed in 4 minutes.
//              H: how many experiments are in the queue?
//              R: 6
//      H: add repair bay alpha left 3 to the top of the schedule

//      H: Describe how to run experiment 5.
//      R: First I get materials. Then I go to bay gamma right 4. Then I load the materials into the bay. Then they are processed. Then I take the results to bay beta left 4. Then they are analyzed.
//      (This can also be displayed in the web app GUI)

//      H: Experiment 9 is like experiment 4
//      R: what is different
//      H: instead of taking the materials to beta left 4 take them to gamma right 5
//      R: Okay, anything else?
//      H: instead of running spectrometry analysis at beta left 4 run component analysis at gamma right 2
//      (Are there any real world domains that we could use the jargon from for this?)
//      This would probably be better suited to the Web app GUI.

//      H: Add new notification
//      R: when should I send it?
//              H: every time an experiment bay breaks
//      R: Okay, what should it contain?
//              H: Tell me the duration of all currently running experiments
//      R: Okay, Anything else?
//      H: Tell me the number of scheduled experiments

  }

    @Test
    public void multiRobotCaddyTest() {
        component.addDictionary("templatedict.dict");
        component.addDictionary("multiRobotCaddy.dict");
        component.addDictionary("multiRobotCaddyGenerated.dict");

        assertTrue(testUtterance("setup demo",
                "INSTRUCT(brad,self,setupscene(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("find the screw",
                "INSTRUCT(brad,shafer,findObject(shafer,VAR0:physobj),{screw(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("find the small gear",
                "INSTRUCT(brad,shafer,findObject(shafer,VAR0:physobj),{smallgear(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("Define new kit repair kit",
                "INSTRUCT(brad,self,defineRecipe(self,\"repair kit\"),{})",
                "brad",
                "self"
        ));

        //what container does it use?
        assertTrue(testUtterance("the container is tool caddy",
                "REPLY(brad,self,val(toolcaddy:physobj),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there is one tool caddy at table a",
                "REPLY(brad,shafer,val(observableAt(1,toolcaddy:physobj,tableA:area)),{})",
                "brad",
                "shafer"
        ));

        //what does it contain
        //TODO:brad: asr for numerals vs strings
        assertTrue(testUtterance("it contains 2 screws",
                "REPLY(brad,self,val(contains(2,screw:property)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there are 2 screws at table a",
                "REPLY(brad,shafer,val(observableAt(2,screw:property,tableA:area)),{})",
                "brad",
                "shafer"
        ));

        //anything else?
        assertTrue(testUtterance("it contains one small gear",
                "REPLY(brad,self,val(contains(1,smallgear:physobj)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there is one small gear at table b",
                "REPLY(brad,shafer,val(observableAt(1,smallgear:physobj,tableB:area)),{})",
                "brad",
                "shafer"
        ));

        //anything else?
        assertTrue(testUtterance("that is all",
                "REPLY(brad,self,val(none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("deliver a repair kit to alpha",
                "INSTRUCT(brad,self,deliver(self,repairkit,VAR0:temilocation),{alpha(VAR0:temilocation),DEFINITE(VAR0:temilocation)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("Define new kit med kit",
                "INSTRUCT(brad,self,defineRecipe(self,medkit),{})",
                "brad",
                "self"
        ));

        //what container does it use?
        assertTrue(testUtterance("it uses a medical caddy",
                "REPLY(brad,self,val(medicalcaddy:physobj),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there is one medical caddy at table a",
                "REPLY(brad,shafer,val(observableAt(1,medicalcaddy:physobj,tableA:area)),{})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("it contains 2 painkillers",
                "REPLY(brad,self,val(contains(2,painkiller:property)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there are 2 painkillers at table b",
                "REPLY(brad,shafer,val(observableAt(2,painkiller:property,tableB:area)),{})",
                "brad",
                "shafer"
        ));

        //anything else?
        assertTrue(testUtterance("it contains one bandage box",
                "REPLY(brad,self,val(contains(1,bandagebox:physobj)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there is one bandage box at table c",
                "REPLY(brad,shafer,val(observableAt(1,bandagebox:physobj,tableC:area)),{})",
                "brad",
                "shafer"
        ));

        assertTrue(testUtterance("it contains one antiseptic",
                "REPLY(brad,self,val(contains(1,antiseptic:physobj)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("there is one antiseptic at table c",
                "REPLY(brad,shafer,val(observableAt(1,antiseptic:physobj,tableC:area)),{})",
                "brad",
                "shafer"
        ));

        //anything else?
        assertTrue(testUtterance("that is all",
                "REPLY(brad,self,val(none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("deliver a med kit to alpha",
                "INSTRUCT(brad,self,deliver(self,medkit,VAR0:temilocation),{alpha(VAR0:temilocation),DEFINITE(VAR0:temilocation)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("ready",
                "REPLY(brad,self,val(ready),{})",
                "brad",
                "self"
        ));
    }
    @Test
    public void sortingDemoTest() {
        component.addDictionary("sortingDemo.dict");
        component.addDictionary("poc4.dict");
        component.addDictionary("assemblyHomophones.dict");

        assertTrue(testUtterance("verify that you can see a pill bottle",
                "INSTRUCT(brad,self,perceiveEntity(self,VAR0:physobj),{pillBottle(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));


        component.injectDictionaryEntry("work area", "RN", "\"work area\":pose", "VAR");
        component.injectDictionaryEntry("work area", "REF", "\"work area\":pose", "DEFINITE");

        component.injectDictionaryEntry("box a", "RN", "\"box a\":pose", "VAR");
        component.injectDictionaryEntry("box a", "REF", "\"box a\":pose", "DEFINITE");

        component.injectDictionaryEntry("box b", "RN", "\"box b\":pose", "VAR");
        component.injectDictionaryEntry("box b", "REF", "\"box b\":pose", "DEFINITE");

        //TODO:brad: semantic type for it?
        assertTrue(testUtterance("Then get it on the work area",
                "INSTRUCT(brad,self,getOn(self,VAR0,VAR1:pose),{it(VAR0),\"work area\"(VAR1:pose),INFOCUS(VAR0),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Then get the pill bottle on the work area",
                "INSTRUCT(brad,self,getOn(self,VAR0:physobj,VAR1:pose),{pillBottle(VAR0:physobj),\"work area\"(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("If the scale reads over 100 grams then get the pill bottle in box a",
                "STATEMENT(brad,self,if(itemWeightGreaterThan(self,100),then(getOn(self,VAR0:physobj,VAR1:pose))),{pillBottle(VAR0:physobj),\"box a\"(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Otherwise put the pill bottle in box b",
                "STATEMENT(brad,self,else(getOn(self,VAR0:physobj,VAR1:pose)),{pillBottle(VAR0:physobj),\"box b\"(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("sort",
                "INSTRUCT(brad,self,sort(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("run sort as a persistent goal",
                "INSTRUCT(brad,self,persistent(self,sort(self)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("start sorting",
                "INSTRUCT(brad,self,controlLoop(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("modify action sort",
                "INSTRUCT(brad,self,modifyInteractive(self,sort(self)),{})",
                "brad",
                "self"
        ));

//        replace turn right with look down

        assertTrue(testUtterance("replace the scale reads over 100 grams with the scale reads over 150 grams",
                "REPLY(brad,self,mod(replace(itemWeightGreaterThan(self,150),itemWeightGreaterThan(self,100)),none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("That is all",
                "REPLY(brad,self,mod(none,none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Aendere Handlung sortieren",
                "INSTRUCT(brad,self,modifyInteractiveGerman(self,sort(self)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("ersetze die wage zeigt ueber 100 gramm durch die wage zeigt ueber 150 gramm",
                "REPLY(brad,self,mod(replace(itemWeightGreaterThan(self,150),itemWeightGreaterThan(self,100)),none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Das ist alles",
                "REPLY(brad,self,mod(none),{})",
                "brad",
                "self"
        ));


    }

    @Test
    public void sortingDemoABBTest() {
        component.addDictionary("sortingDemo.dict");
        component.addDictionary("poc4.dict");
        component.addDictionary("assemblyHomophones.dict");

        assertTrue(testUtterance("verify that you can see a part",
                "INSTRUCT(brad,self,perceiveEntity(self,VAR0:physobj),{part(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));


        component.injectDictionaryEntry("conveyor", "RN", "conveyor:pose", "VAR");
        component.injectDictionaryEntry("conveyor", "REF", "conveyor:pose", "DEFINITE");

        component.injectDictionaryEntry("table a", "RN", "\"table a\":pose", "VAR");
        component.injectDictionaryEntry("table a", "REF", "\"table a\":pose", "DEFINITE");

        component.injectDictionaryEntry("table b", "RN", "\"table b\":pose", "VAR");
        component.injectDictionaryEntry("table b", "REF", "\"table b\":pose", "DEFINITE");

        assertTrue(testUtterance("go to pose conveyor",
                "INSTRUCT(brad,self,gotopose(self,VAR0:pose),{conveyor(VAR0:pose),DEFINITE(VAR0:pose)})",
                "brad",
                "self"
        ));


        //TODO:brad: semantic type for it?
        assertTrue(testUtterance("Then get it on the conveyor",
                "INSTRUCT(brad,self,getOn(self,VAR0,VAR1:pose),{it(VAR0),conveyor(VAR1:pose),INFOCUS(VAR0),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Then get the part on the conveyor",
                "INSTRUCT(brad,self,getOn(self,VAR0:physobj,VAR1:pose),{part(VAR0:physobj),conveyor(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("If the part is longer than 75 mm then get the part on table a",
                "STATEMENT(brad,self,if(partLengthGreaterThan(self,75),then(getOn(self,VAR0:physobj,VAR1:pose))),{part(VAR0:physobj),\"table a\"(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Otherwise put the part in table b",
                "STATEMENT(brad,self,else(getOn(self,VAR0:physobj,VAR1:pose)),{part(VAR0:physobj),\"table b\"(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("sort",
                "INSTRUCT(brad,self,sort(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("run sort as a persistent goal",
                "INSTRUCT(brad,self,persistent(self,sort(self)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("start sorting",
                "INSTRUCT(brad,self,controlLoop(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("modify action sort",
                "INSTRUCT(brad,self,modifyInteractive(self,sort(self)),{})",
                "brad",
                "self"
        ));

//        replace turn right with look down

        assertTrue(testUtterance("replace the part is longer than 75 mm with the part is longer than 100 mm",
                "REPLY(brad,self,mod(replace(partLengthGreaterThan(self,100),partLengthGreaterThan(self,75)),none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("That is all",
                "REPLY(brad,self,mod(none,none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Aendere Handlung sortieren",
                "INSTRUCT(brad,self,modifyInteractiveGerman(self,sort(self)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("ersetze die wage zeigt ueber 100 gramm durch die wage zeigt ueber 150 gramm",
                "REPLY(brad,self,mod(replace(itemWeightGreaterThan(self,150),itemWeightGreaterThan(self,100)),none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Das ist alles",
                "REPLY(brad,self,mod(none),{})",
                "brad",
                "self"
        ));


    }

    @Test
    public void onRobotScrewingTest() {
        component.addDictionary("poc4.dict");
        component.addDictionary("assemblyHomophones.dict");

        component.injectDictionaryEntry("conveyor", "RN", "conveyor", "VAR");
        component.injectDictionaryEntry("conveyor", "REF", "conveyor", "DEFINITE");

        component.injectDictionaryEntry("work area", "RN", "\"work area\"", "VAR");
        component.injectDictionaryEntry("conveyor", "REF", "\"work area\"", "DEFINITE");

        component.injectDictionaryEntry("screw feeder", "RN", "\"screw feeder\"", "VAR");
        component.injectDictionaryEntry("screw feeder", "REF", "\"screw feeder\"", "DEFINITE");

        component.injectDictionaryEntry("screw feeder", "POSE", "\"screw feeder\"", "VAR");

        assertTrue(testUtterance("define new screw type M3",
                "INSTRUCT(brad,self,defineScrewType(self,m3),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("define new screw type deep M3",
                "INSTRUCT(brad,self,defineScrewType(self,deepM3),{})",
                "brad",
                "self"
        ));


        //    TRACS: How long are screws of this type?
        assertTrue(testUtterance("7 millimeters",
                "REPLY(brad,self,val(7,mm),{})",
                "brad",
                "self"
        ));

        //    TRACS: What is the target torque?
        assertTrue(testUtterance("150 millinewton meters",
                "REPLY(brad,self,val(150,mNm),{})",
                "brad",
                "self"
        ));


        //Where can I find m3 screws?
        assertTrue(testUtterance("pose screw feeder",
                "REPLY(brad,self,pose(VAR0),{\"screw feeder\"(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //VIDEO B.1
//        Human: Define new item NF32-SV circuit breaker
        assertTrue(testUtterance("define new item NF32-SV",
                "INSTRUCT(brad,self,defineItem(self,nf32sv),{})",
                "brad",
                "self"
        ));

        component.injectDictionaryEntry("nf32-sv", "RN", "nf32-sv", "VAR");
        component.injectDictionaryEntry("nf32-sv", "DESC", "nf32-sv","");

        //which Cognex job is used to detect its hole
        assertTrue(testUtterance("job circuit breaker face",
                "REPLY(brad,self,job(cbDet),{})",
                "brad",
                "self"
        ));

//                Robot: "Okay. Does it have any other features?"
//        Human: It has 2 M3 screw holes on top.

        //VIDEO B.2


        assertTrue(testUtterance("I will teach you how to assemble an NF32-SV",
                "STATEMENT(brad,self,will(teach(brad,self,to(assemble(VAR0)))),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("that is how you assemble an NF32-SV",
                "STATEMENT(brad,self,endTeaching(self,assemble(VAR0)),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("first go to pose conveyor",
                "INSTRUCT(brad,self,goToCameraPose(self,VAR0),{conveyor(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        //        Human: Then verify that you can see a.
        assertTrue(testUtterance("then verify that you can see an NF32-SV",
                "INSTRUCT(brad,self,perceiveEntity(self,VAR0),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Then get it on the work area",
                "INSTRUCT(brad,self,getOn(self,VAR0,VAR1),{it(VAR0),\"work area\"(VAR1),INFOCUS(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then search for 2 mounting holes",
                "INSTRUCT(brad,self,observeDescriptor(self,deepM3Hole,2),{})",
                "brad",
                "self"
        ));

//        Human: Then screw m3 screws into the two holes on top

        assertTrue(testUtterance("screw a deep M3 into the bottom mounting hole",
                "INSTRUCT(brad,self,screwIn(self,deepM3,VAR0),{deepM3Hole(VAR0),bottom(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("screw an deep M3  into the top mounting hole",
                "INSTRUCT(brad,self,screwIn(self,deepM3,VAR0),{deepM3Hole(VAR0),top(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("Then get the NF32-SV on the conveyor",
                "INSTRUCT(brad,self,getOn(self,VAR0,VAR1),{nf32sv(VAR0),conveyor(VAR1),DEFINITE(VAR0),DEFINITE(VAR1)})",
                "brad",
                "self"
        ));

        //VIDEO B.3
        assertTrue(testUtterance("assemble a NF32-SV ",
                "INSTRUCT(brad,self,assemble(self,VAR0),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("assemble a NF-32SV ",
                "INSTRUCT(brad,self,assemble(self,VAR0),{nf32sv(VAR0),DEFINITE(VAR0)})",
                "brad",
                "self"
        ));



//        replace search for 2 depp m3 holes with search for 2 m3 holes
//        replace screw a deep M3 into the bottom mounting hole with screw an M3 screw into the left M3 hole
//        replace screw a deep M3 into the top mounting hole with screw an M3 screw into the right M3 hole

    }

    @Test
    public void yumiFoodOrderingTest() {

        component.addDictionary("foodOrdering.dict");
//        component.addDictionary("foodOrderingHomophones.dict");
        component.addDictionary("yumiFoodOrderingGenerated.dict");
        component.setUpdateAddressHistory(false);

//
//        Basic NL commands

        assertTrue(testUtterance("right arm go to prep area",
                "INSTRUCT(brad,self,goTo(rightArm:yumi,VAR0:area),{\"prep area\"(VAR0:area),DEFINITE(VAR0:area)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("right arm do you see a bell pepper",
                "INSTRUCT(brad,self,doYouSee(rightArm:yumi,VAR0:physobj),{\"bell pepper\"(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("right arm look for a bell pepper",
                "INSTRUCT(brad,self,lookFor(rightArm:yumi,VAR0:physobj),{\"bell pepper\"(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("right arm pick up the bell pepper",
                "INSTRUCT(brad,self,pickUp(rightArm:yumi,VAR0:physobj),{\"bell pepper\"(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("right arm pick it up",
                "INSTRUCT(brad,self,pickUp(rightArm:yumi,VAR0:physobj),{it(VAR0:physobj),INFOCUS(VAR0:physobj)})",
                "brad",
                "self"
        ));
//
//        H: then go to cook top
        assertTrue(testUtterance("right arm go to cook top",
                "INSTRUCT(brad,self,goTo(rightArm:yumi,VAR0:area),{\"cook top\"(VAR0:area),DEFINITE(VAR0:area)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then right arm put down the bell pepper",
                "INSTRUCT(brad,self,putDown(rightArm:yumi,VAR0:physobj),{\"bell pepper\"(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("pause",
                "INSTRUCT(brad,self,suspendCurrentGoal(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("What is your current task",
                "INSTRUCT(brad,self,describeCurrentGoal(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("what are your pending tasks",
                "INSTRUCT(brad,self,describePendingGoals(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("cancel pending tasks",
                "INSTRUCT(brad,self,cancelAllPendingGoals(self),{})",
                "brad",
                "self"
        ));

//        H: I cleaned up
        assertTrue(testUtterance("reset",
                "INSTRUCT(brad,self,resetDomain(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("define new item southwest bowl",
                "INSTRUCT(brad,self,defineItem(self,\"southwest bowl\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then get a bell pepper to the cook top",
                "INSTRUCT(brad,self,getTo(self,VAR0:physobj,VAR1:area),{\"bell pepper\"(VAR0:physobj),\"cook top\"(VAR1:area),DEFINITE(VAR0:physobj),DEFINITE(VAR1:area)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then right arm saute the bell pepper for 20 seconds ",
                "INSTRUCT(brad,self,saute(rightArm:yumi,VAR0:physobj,val(20,seconds)),{\"bell pepper\"(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("the get a bell pepper in the serving box",
                "INSTRUCT(brad,self,getOn(self,VAR0:physobj,VAR1:physobj),{\"bell pepper\"(VAR0:physobj),\"serving box\"(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then right arm cook a corn for 15 seconds",
                "INSTRUCT(brad,self,cook(rightArm:yumi,VAR0:physobj,val(15,seconds)),{corn(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then left arm drizzle on the sauce",
                "INSTRUCT(brad,self,drizzle(leftArm:yumi,VAR0:physobj),{sauce(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("then left arm drizzle on chipotle sauce",
                "INSTRUCT(brad,self,drizzle(leftArm:yumi,VAR0:physobj),{\"chipotle sauce\"(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("that is how you prepare a southwest bowl",
                "INSTRUCT(brad,self,endItemDefinition(self,prepare\"southwest bowl\"(self,\"southwest bowl\")),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("prepare a southwest bowl",
                "INSTRUCT(brad,self,prepare\"southwest bowl\"(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("describe how to prepare a southwest bowl",
                "INSTRUCT(brad,self,describe(self,how(to(prepare\"southwest bowl\"(self)))),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("define new item by analogy puerto rican bowl",
                "INSTRUCT(brad,self,defineItemByAnalogy(self,\"puerto rican bowl\"),{})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("item southwest bowl",
                "REPLY(brad,self,item(\"southwest bowl\"),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("southwest bowl",
                "REPLY(brad,self,item(\"southwest bowl\"),{})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("replace right arm saute the carrot for 30 seconds with right arm saute the chicken for 45 seconds",
                "REPLY(brad,self,mod(replace(saute(rightArm:yumi,VAR1:physobj,val(45,seconds)),saute(rightArm:yumi,VAR0:physobj,val(30,seconds))),none()),{carrot(VAR0:physobj),chicken(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("remove the bell pepper",
                "REPLY(brad,self,mod(removeLocalVar(VAR0:physobj),none()),{\"bell pepper\"(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("add get a plantain in the serving box after get the chicken in the serving box",
                "REPLY(brad,self,mod(insert(getOn(self,VAR0:physobj,VAR1:physobj)),after(getOn(self,VAR2:physobj,VAR3:physobj))),{plantain(VAR0:physobj),\"serving box\"(VAR1:physobj),chicken(VAR2:physobj),\"serving box\"(VAR3:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj),DEFINITE(VAR2:physobj),DEFINITE(VAR3:physobj)})",
                "brad",
                "self"
        ));
//-
//suspend current task
        assertTrue(testUtterance("suspend current task",
                "INSTRUCT(brad,self,suspendCurrentGoal(self),{})",
                "brad",
                "self"
        ));
//-
//define new ingredient plantain
        assertTrue(testUtterance("now define new ingredient plantain",
                "INSTRUCT(brad,self,urgentPriority(self,defineIngredient(self,plantain)),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("area pantry",
                "REPLY(brad,self,area(VAR0:area),{pantry(VAR0:area),DEFINITE(VAR0:area)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("detect plantain",
                "REPLY(brad,self,job(plantain),{})",
                "brad",
                "self"
        ));


        assertTrue(testUtterance("resume previous task",
                "INSTRUCT(brad,self,resumeCurrentGoal(self),{})",
                "brad",
                "self"
        ));

        //add get a plantain in the serving box after get the chicken in the serving box

        assertTrue(testUtterance("no more differences",
                "REPLY(brad,self,mod(none,none),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("prepare a puerto rican bowl",
                "INSTRUCT(brad,self,prepare\"puerto rican bowl\"(self),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("option aoli",
                "REPLY(brad,self,option(VAR0:physobj),{aoli(VAR0:physobj),DEFINITE(VAR0:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("here it is",
                "REPLY(brad,self,got(ingredient),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("do you know how to get the carrot in the serving box",
                "QUESTION(brad,self,knowHow(self,to(getOn(self,VAR0:physobj,VAR1:physobj))),{carrot(VAR0:physobj),\"serving box\"(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));

        //TODO:brad: this seems to not work, but do you know how does?
        assertTrue(testUtterance("can you get the carrot in the serving box",
                "QUESTION(brad,self,capableOf(self,getOn(self,VAR0:physobj,VAR1:physobj)),{carrot(VAR0:physobj),\"serving box\"(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));



        //TODO:brad: only describes act specs
        assertTrue(testUtterance("describe how to get the corn in the serving box",
                "INSTRUCT(brad,self,describe(self,how(to(getOn(self,VAR0:physobj,VAR1:physobj)))),{corn(VAR0:physobj),\"serving box\"(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("left arm open gripper",
                "INSTRUCT(brad,self,openGripper(leftArm:yumi),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("left arm close gripper",
                "INSTRUCT(brad,self,closeGripper(leftArm:yumi),{})",
                "brad",
                "self"
        ));


//        assertTrue(testUtterance("pickup the corn",
//                "INSTRUCT(brad,self,describe(self,how(to(getOn(self,VAR0:physobj,VAR1:physobj)))),{corn(VAR0:physobj),\"serving box\"(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
//                "brad",
//                "self"
//        ));
//
//        assertTrue(testUtterance("pickup the corn from prep area",
//                "INSTRUCT(brad,self,describe(self,how(to(getOn(self,VAR0:physobj,VAR1:physobj)))),{corn(VAR0:physobj),\"serving box\"(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
//                "brad",
//                "self"
//        ));



    }

}
