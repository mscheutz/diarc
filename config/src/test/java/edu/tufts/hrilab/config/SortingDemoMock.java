/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.scale.MockScaleComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SortingDemoMock extends DiarcConfiguration {
    protected static Logger log = LoggerFactory.getLogger(SortingDemoMock.class);
    public SimSpeechRecognitionComponent simspeech;
    public ai.thinkingrobots.mtracs.interfaces.CR800ComponentInterface assista;
    public ai.thinkingrobots.mtracs.interfaces.PLCComponentInterface plc;

    @Override
    public void runConfiguration(){
        assista = createInstance(ai.thinkingrobots.mtracs.mock.MockCR800Component.class, "-config mtracs/assista.json");
        plc = createInstance(ai.thinkingrobots.mtracs.mock.MockPLCComponent.class);
        createInstance(MockScaleComponent.class);

        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);
        createInstance(TLDLParserComponent.class, "-dict poc4.dict -dict assemblyHomophones.dict -dict sortingDemo.dict");
        createInstance(ReferenceResolutionComponent.class);
        createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class);
        createInstance(SimpleNLGComponent.class);
        createInstance(SimSpeechProductionComponent.class, "");

        createInstance(GoalManagerComponent.class,
//                        "-editor " +
//                        "-beliefg " +
                        "-beliefinitfile demos.pl " +
                        "-beliefinitfile agents/agents.pl " +
                        "-beliefinitfile demos/poc4.pl " +
                        "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector " +
                        "-dbfileDir / " +

                        "-asl config/edu/tufts/hrilab/action/asl/core.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/handleSemantics.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlu.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlg.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/demos/sortingDemo.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/modifyActionInteractive.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/modifyActionInteractiveGerman.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/circuitBreakerScrewingActions.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/assistaCognex.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/mtracs/conveyor.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/mtracs/cr800Base.asl " +

                        "-goal listen(self:agent) "
        );


        //TODO:brad: test args case
//        simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker brad -config sortingDemo.simspeech");
        simspeech = createInstance(SimSpeechRecognitionComponent.class, "-nogui -speaker brad -config sortingDemo.simspeech");

    }

}
