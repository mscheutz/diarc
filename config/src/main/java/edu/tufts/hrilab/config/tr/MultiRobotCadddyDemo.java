/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.tr;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fetch.MockFetchItComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.asr.sphinx4.Sphinx4Component;
import edu.tufts.hrilab.vision.MockVisionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.spot.MockSpotComponent;

public class MultiRobotCadddyDemo extends DiarcConfiguration {

    protected static Logger log = LoggerFactory.getLogger(MultiRobotCadddyDemo.class);

    static public SimSpeechRecognitionComponent simSpeechRec;

    // start the configuration
    @Override
    public void runConfiguration() {
        boolean useMock = true;
        boolean useMockTemi = true;
        boolean useMockVision = true;
        boolean useMockMovebase = true;
        boolean useMockManipulation = true;
        boolean useSphinx = false;
        boolean useSpot = true;

        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

        createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict multiRobotCaddy.dict -dict multiRobotCaddyHomophones.dict");

        createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

        createInstance(ReferenceResolutionComponent.class);

        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);

        createInstance(SimpleNLGComponent.class);

        if (useSphinx) {
            createInstance(Sphinx4Component.class, "-grammar heterogeneousAgents -nlp -speaker tyler -listener");
            //createInstance(edu.tufts.hrilab.sphinx4.Sphinx4Component.class, "-grammar learnAssemble -nlp -speaker tyler -listener andy");
        }

        if (useMockVision) {
            createInstance(MockVisionComponent.class, "-groups agent:fetch:fetch -belief -sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");
        } else {
           // createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-groups agent:fetch:fetch -capture pcd.xml -height 1.3");
//           createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-groups agent:fetch:fetch -cameraFrame head_camera_rgb_optical_frame -capture fetch.xml");
        }

        if (useMockManipulation) {
            createInstance(MockFetchItComponent.class, "-groups agent:fetch:fetch -simExecTime");
            createInstance(com.tf.MockTFComponent.class, "-groups agent:fetch:fetch");
        } else {
//            createInstance(com.fetch.FetchItComponent.class, "-groups agent:fetch:fetch -startPose");
//            createInstance(com.tf.TFComponent.class, "-groups agent:fetch:fetch");
        }

        createInstance(com.movebase.map.MapComponent.class, "-kb_name movebaselocation -refs refs/locationRefsMultiRobotCaddy.json -groups agent:fetch:fetch");
        if (useMockMovebase) {
            createInstance(com.movebase.MockMoveBaseComponent.class, "-groups agent:fetch:fetch -simExecTime");
        } else {
//            createInstance(com.movebase.MoveBaseComponent.class, "-groups agent:fetch:fetch");
        }

        if(useMockTemi){
            createInstance(com.temiv3.MockTemiV3Component.class, "-mapDescriptor temilocation -groups agent:temi:temi");
        }

        if (useMock) {
            createInstance(com.simspeech.SimSpeechProductionComponent.class, "");
            if (useSpot) {
                createInstance(com.spot.MockSpotNavGraphComponent.class, "-mapDescriptor spotlocation -refs spotLocationRefs.json -groups agent:spot:spot -simExecTime");
                createInstance(MockSpotComponent.class, "-groups agent:spot:spot");
            }
        } else {
            if (useSpot) {
//                createInstance(com.spot.SpotComponent.class, "-groups agent:spot:spot");
            }
//            createInstance(com.tts.MaryTTSComponent.class, "-groups agent:fetch:fetch");
//            createInstance(com.fetch.FetchComponent.class, "-groups agent:fetch:fetch");
//            createInstance(com.movebase.MoveBaseComponent.class, "-mapDescriptor movebaselocation -refs refs/locationRefsMultiRobotCaddy.json -groups agent:fetch:fetch");
//            //todo: spot component
        }

        String goalManagerArgs = "-editor "
                + "-beliefg "
                + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
                + " -beliefinitfile demos.pl "
                + "-beliefinitfile agents/multiRobotCaddyAgents.pl "
                + "-beliefinitfile demos/multiRobotCaddy/multiRobotCaddy.pl "
                + "-beliefinitfile demos/multiRobotCaddy/environmentFacts.pl "
                + "-beliefinitfile demos/multiRobotCaddy/fetchFacts.pl "
                + "-beliefinitfile demos/multiRobotCaddy/temiFacts.pl "
                + "-nomemorymanager "
                + "-dbfile core.asl "
                + "-dbfile dialogue/nlg.asl "
                + "-dbfile dialogue/nlu.asl "
                + "-dbfile dialogue/handleSemantics.asl "
                + "-dbfile domains/tr/multiRobotCaddyRecipe.asl "
                + "-dbfile demos/multiRobotCaddy.asl "
                + "fetchit/fetchit_manipulation.asl "
                + "vision.asl "
                + "navigation.asl "
                + "brad:human:demos/multiRobotCaddyHuman.asl "
                + "-performancefile multiRobotCaddy.json "
                + "-goal listen(self:agent)";

        if (useSpot) {
            goalManagerArgs += "-beliefinitfile demos/multiRobotCaddy/spotFacts.pl ";
        }

        createInstance(GoalManagerComponent.class,
                goalManagerArgs
        );

        //brad: this is last so the GUI window is on top
        simSpeechRec = createInstance(SimSpeechRecognitionComponent.class,
                "-config multiRobotCaddy.simspeech -speaker tyler");
    }

}
