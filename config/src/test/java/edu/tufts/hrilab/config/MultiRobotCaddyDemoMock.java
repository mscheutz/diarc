/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fetch.MockFetchItComponent;
import edu.tufts.hrilab.firebase.MockFirebaseConnectionComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.spot.MockSpotComponent;
import edu.tufts.hrilab.spot.MockSpotNavGraphComponent;
import edu.tufts.hrilab.temi.MockTemiV3Component;
import edu.tufts.hrilab.vision.MockVisionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class MultiRobotCaddyDemoMock extends DiarcConfiguration {
    protected static Logger log = LoggerFactory.getLogger(MultiRobotCaddyDemoMock.class);
    static public SimSpeechRecognitionComponent simSpeechRec;
    static public MockVisionComponent vision;


    // start the configuration
    @Override
    public void runConfiguration() {
        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

        createInstance(TLDLParserComponent.class, "-dict templatedict.dict -dict multiRobotCaddy.dict -dict multiRobotCaddyHomophones.dict");

        createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

        createInstance(ReferenceResolutionComponent.class);

        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);

        createInstance(SimpleNLGComponent.class);

        vision = createInstance(MockVisionComponent.class, "-groups agent:fetch:fetch -belief -sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");

        createInstance(MockFetchItComponent.class, "-groups agent:fetch:fetch ");
        createInstance(edu.tufts.hrilab.tf.MockTFComponent.class, "-groups agent:fetch:fetch");

        createInstance(edu.tufts.hrilab.map.MapComponent.class, "-kb_name movebaselocation -refs refs/locationRefsMultiRobotCaddy.json -groups agent:fetch:fetch");
        createInstance(MockMoveBaseComponent.class, "-groups agent:fetch:fetch ");

        createInstance(SimSpeechProductionComponent.class, "");
        createInstance(MockSpotNavGraphComponent.class, "-mapDescriptor spotlocation -refs spotLocationRefs.json -groups agent:spot:spot ");
        createInstance(MockSpotComponent.class, "-groups agent:spot:spot");
        createInstance(MockTemiV3Component.class, "-mapDescriptor temilocation -groups agent:temi:temi");

        String goalManagerArgs = "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
                + " -beliefinitfile demos.pl "
                + "-beliefinitfile agents/multiRobotCaddyAgents.pl "
                + "-beliefinitfile demos/multiRobotCaddy/multiRobotCaddy.pl "
                + "-beliefinitfile demos/multiRobotCaddy/environmentFacts.pl "
                + "-beliefinitfile demos/multiRobotCaddy/fetchFacts.pl "
                + "-beliefinitfile demos/multiRobotCaddy/temiFacts.pl "
                + "-beliefinitfile demos/multiRobotCaddy/spotFacts.pl "
                + "-nomemorymanager "
                + "-asl core.asl "
                + "-asl dialogue/nlg.asl "
                + "-asl dialogue/nlu.asl "
                + "-asl dialogue/handleSemantics.asl "
                + "-asl domains/tr/multiRobotCaddyRecipe.asl "
                + "-asl demos/multiRobotCaddy.asl "
                + "-asl fetchit/fetchit_manipulation.asl "
                + "-asl manipulation.asl "
                + "-asl vision.asl "
                + "-asl navigation.asl "
                + "-asl brad:human:demos/multiRobotCaddyHuman.asl "
                + "-performancefile multiRobotCaddy.json "
                + "-goal listen(self:agent)";
//        if (showGUI) {
//            gmArgs += "-editor ";
//        }

        createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class,
                goalManagerArgs
        );

//        createInstance(MockFirebaseConnectionComponent.class, "-dialogue -emulator -firebaseGroup TuftsDemo -action -dbaction planned -dbfile /config/edu/tufts/hrilab/action/asl/demos/multiRobotCaddy.asl -dbfile /config/edu/tufts/hrilab/action/asl/domains/multiRobotCaddy.asl");

        //brad: this is last so the GUI window is on top
        simSpeechRec = createInstance(SimSpeechRecognitionComponent.class,
                "-config multiRobotCaddy.simspeech -nogui -speaker evan");
//        if (!showGUI) {
//            simRecArgs += "-nogui";
//        }

    }

    public static void main(String[] args){
        MultiRobotCaddyDemoMock demoConfig = new MultiRobotCaddyDemoMock();
        demoConfig.runConfiguration();
    }

}
