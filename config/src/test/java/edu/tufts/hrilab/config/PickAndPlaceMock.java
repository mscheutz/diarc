package edu.tufts.hrilab.config;

import ai.thinkingrobots.mtracs.interfaces.CR800ComponentInterface;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;

public class PickAndPlaceMock extends DiarcConfiguration {
    private CR800ComponentInterface assista;
    public SimSpeechRecognitionComponent simspeech;
    String languageGenerationASLFile = "config/edu/tufts/hrilab/action/asl/demos/pickAndPlace/pickAndPlaceLanguageGeneration.asl";

    @Override
    public void runConfiguration() {
        assista = DiarcComponent.createInstance(ai.thinkingrobots.mtracs.mock.MockCR800Component.class, "-config ai/thinkingrobots/mtracs/config/robotone.json -groups agent:robotone:agent");
        DiarcComponent.createInstance(ai.thinkingrobots.mtracs.mock.MockOnRobotScrewingComponent.class, "-groups agent:robotone:agent");

        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);
        createInstance(edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent.class, "");
        createInstance(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent.class, "-pragrules demos.prag");
        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class, "");
        createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");

        String hybridParserArgs = "-tldl pickAndPlace.dict -cache -cacheName pickAndPlace2024 -cachePersist true";
        createInstance(SimpleNLGComponent.class);
        createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, hybridParserArgs);

        String goalManagerArgs=
                "-beliefg " +
                "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                "-beliefinitfile demos.pl " +
                        "-beliefinitfile agents/onRobotScrewingAgents.pl " +
                        "-beliefinitfile demos/onRobotScrewing.pl " +
                        "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector " +
//                        "-selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector " +

                        "-dbfileDir / " +
                        "-asl config/edu/tufts/hrilab/action/asl/core.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/handleSemantics.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlu.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlg.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/cognex.asl " +

                        //todo: should move these asl files and sort into ones that are/are not specific to screwing most/all of this is actually general to the robot arm demos w the cognex and M arms
                        "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/assistaCognex.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/cognexArmActions.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/demos/pickAndPlace/pickAndPlaceInit.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/pickAndPlace/pickAndPlaceActions.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/pickAndPlace/cognexArmUtil.asl " +
                        "-asl " + languageGenerationASLFile + " " +

                        "-asl config/edu/tufts/hrilab/action/asl/mtracs/cr800Base.asl " +

                        "-goal listen(self) ";

        //goalManagerArgs +="-editor";

        createInstance(edu.tufts.hrilab.action.GoalManagerComponent.class, goalManagerArgs);

        simspeech = DiarcComponent.createInstance(SimSpeechRecognitionComponent.class, "-speaker pete -addressee robotone -config pickAndPlace.simspeech -nogui");
    }
}
