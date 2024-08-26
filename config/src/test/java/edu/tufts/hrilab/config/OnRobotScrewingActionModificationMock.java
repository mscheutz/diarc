/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class OnRobotScrewingActionModificationMock extends DiarcConfiguration {
    protected static Logger log = LoggerFactory.getLogger(OnRobotScrewingActionModificationMock.class);
    public SimSpeechRecognitionComponent simspeech;
    public ai.thinkingrobots.mtracs.interfaces.CR800ComponentInterface assista;
    public ai.thinkingrobots.mtracs.interfaces.CR800ComponentInterface rv4fr;
    public ai.thinkingrobots.mtracs.interfaces.PLCComponentInterface plc;
    private  boolean firebase;
    private  boolean test;
    public OnRobotScrewingActionModificationMock(boolean firebase, boolean test) {
        this.firebase=firebase;
        this.test=test;
    }

    @Override
    public void runConfiguration() {

        assista = DiarcComponent.createInstance(ai.thinkingrobots.mtracs.mock.MockCR800Component.class, "-config ai/thinkingrobots/mtracs/config/robotone.json -groups agent:robotone:agent");
        rv4fr = DiarcComponent.createInstance(ai.thinkingrobots.mtracs.mock.MockCR800Component.class, "-config ai/thinkingrobots/mtracs/config/robottwo.json -groups agent:robottwo:agent");
        DiarcComponent.createInstance(ai.thinkingrobots.mtracs.mock.MockOnRobotScrewingComponent.class, "-groups agent:robotone:agent");
        DiarcComponent.createInstance(ai.thinkingrobots.mtracs.mock.MockOnRobotScrewingComponent.class, "-groups agent:robottwo:agent");
        plc = DiarcComponent.createInstance(ai.thinkingrobots.mtracs.mock.MockPLCComponent.class, "-groups agent:robotone:agent agent:robottwo:agent");

        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);
        createInstance(TLDLParserComponent.class, "-dict poc4.dict -dict assemblyHomophones.dict ");
        createInstance(ReferenceResolutionComponent.class, "");
        createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class, "");
        createInstance(SimpleNLGComponent.class, "");

        if (firebase) {
            createInstance(edu.tufts.hrilab.firebase.DesktopFirebaseConnectionComponent.class, "-nlp -firebaseGroup tr_emulated -dialogue -emulator -writeASL " +
                    "-dbfile /config/edu/tufts/hrilab/action/asl/demos/assemble.asl " +
                    "-dbfile /config/edu/tufts/hrilab/action/asl/mtracs/conveyor.asl " +
                    "-dbfile /config/edu/tufts/hrilab/action/asl/demos/screwIn.asl " +
                    "-dbfile /config/edu/tufts/hrilab/action/asl/mtracs/cr800Base.asl");
        } else {
            createInstance(SimSpeechProductionComponent.class, "");
        }

        String goalManagerArgs=
                "-beliefinitfile demos.pl agents/onRobotScrewingAgents.pl demos/onRobotScrewing.pl " +
                "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector " +

                "-dbfileDir / " +
                "-asl config/edu/tufts/hrilab/action/asl/core.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/dialogue/handleSemantics.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlu.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlg.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/cognex.asl " +

                "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/assemble.asl " +

                "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/assistaCognex.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/onRobotScrewing.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/circuitBreakerScrewingActions.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/circuitBreakerScrewingInit.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/screwIn.asl "+

                "-asl config/edu/tufts/hrilab/action/asl/mtracs/conveyor.asl " +
                "-asl config/edu/tufts/hrilab/action/asl/mtracs/cr800Base.asl " +

                //TODO:brad this has to come after the agents file
                "-goal listen(self:agent) ";

        if(!test){
            goalManagerArgs +="-editor -beliefg ";
        }

        createInstance(GoalManagerComponent.class, goalManagerArgs);

        if(test){
            simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker james -addressee robotone:agent -config onRobotScrewing.simspeech -nogui");
        } else {
            simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker james  -addressee robotone:agent -config onRobotScrewing.simspeech");
        }

    }

    public static void main(String[] args) {
        boolean firebase = false;
        boolean test = false;
        OnRobotScrewingActionModificationMock config= new OnRobotScrewingActionModificationMock(firebase, test);
        config.runConfiguration();
    }
}