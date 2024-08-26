package edu.tufts.hrilab.config.tr;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.abb.MockYumiComponent;
import edu.tufts.hrilab.abb.PickAndPlaceLLM;
import edu.tufts.hrilab.abb.RequestFromHumanComponent;
import edu.tufts.hrilab.abb.consultant.area.ABBAreaConsultant;
import edu.tufts.hrilab.cognex.consultant.CognexConsultant;
import edu.tufts.hrilab.abb.consultant.item.ItemConsultant;
import edu.tufts.hrilab.abb.consultant.location.ABBLocationConsultant;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
//import ai.thinkingrobots.abb.RWSRobotComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;

public class PickAndPlaceLLMConfig extends DiarcConfiguration {
    protected static Logger log = LoggerFactory.getLogger(PickAndPlaceLLMConfig.class);
    static public SimSpeechRecognitionComponent simSpeechRec;
    public MockYumiComponent leftArm;
    public MockYumiComponent rightArm;
    String service = "llamahf";
    String model = "Meta-Llama-3-70B-Instruct";

    @Override
    public void runConfiguration() {
        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

        leftArm = createInstance(MockYumiComponent.class, "-groups agent:leftArm:yumi");
        rightArm = createInstance(MockYumiComponent.class, "-groups agent:rightArm:yumi");
        //RWSRobotComponent leftArm = DiarcComponent.createInstance(RWSRobotComponent.class, "-basePath http://192.168.125.1 -task T_ROB_L -mainModule module1 -instructionVar instruction -groups agent:leftArm:yumi");
        //RWSRobotComponent rightArm = DiarcComponent.createInstance(RWSRobotComponent.class, "-basePath http://192.168.125.1 -task T_ROB_R -mainModule module1 -instructionVar instruction -groups agent:rightArm:yumi");

        CognexConsultant cognexConsultant = new CognexConsultant();
        try {
            TRADE.registerAllServices(cognexConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "agent:human:mobileManipulator", "physobj")));
        } catch (TRADEException e) {
            log.error("error registering cognex consultant in DIARC config", e);
        }
        leftArm.setCognexConsultant(cognexConsultant);
        rightArm.setCognexConsultant(cognexConsultant);

        createInstance(RequestFromHumanComponent.class, "-groups agent:human:mobileManipulator");

        ItemConsultant itemConsultant = new ItemConsultant();
        try {
            TRADE.registerAllServices(itemConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "item")));
        } catch (TRADEException e) {
            log.error("error registering item consultant in DIARC config", e);
        }

        ABBAreaConsultant areaConsultant = new ABBAreaConsultant();
        try {
            TRADE.registerAllServices(areaConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "area")));
        } catch (TRADEException e) {
            log.error("error registering area consultant in DIARC config", e);
        }

        ABBLocationConsultant locationConsultant = new ABBLocationConsultant();
        try {
            TRADE.registerAllServices(locationConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "agent:human:mobileManipulator", "location")));
        } catch (TRADEException e) {
            log.error("error registering location consultant in DIARC config" + e);
        }

        DiarcComponent.createInstance(LLMComponent.class, String.format("-service %s -model %s -temperature 0.6", service, model));
        createInstance(PickAndPlaceLLM.class, "-nluPrompt pickAndPlace/nlu/pickAndPlaceActionSemanticTranslationYumi -outputLanguage ja");
        createInstance(edu.tufts.hrilab.slug.parsing.llm.LLMParserComponent.class, "-service pickAndPlaceLLMParser");
        //createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, "-tldl foodOrdering.dict -tldl yumiFoodOrderingHomophones.dict -patternMatching -llm pickAndPlaceLLMParser");

        createInstance(ReferenceResolutionComponent.class);
        createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
        createInstance(DialogueComponent.class, "");
        //createInstance(SimpleNLGComponent.class, "");

        createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");

        String gmArgs =
                "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager "
                        + "-diarcagent leftArm:yumi right:yumi human:mobileManipulator "
                        + "-beliefinitfile demos.pl "
                        + "-beliefinitfile agents/yumiFoodOrderingAgents.pl "
                        + "-beliefinitfile demos/yumiFoodOrdering.pl "
                        + "-beliefinitfile demos/yumiPickAndPlaceLLMHardcodedFacts.pl "
                        + "-selector edu.tufts.hrilab.action.selector.TypedActionSelector "
                        + "-asl core.asl "
                        + "-asl dialogue/nlg.asl "
                        + "-asl dialogue/nlu.asl "
                        + "-asl dialogue/handleSemantics.asl "
                        + "-asl demos/foodOrdering/prepareFood.asl "
                        + "-asl demos/foodOrdering/prepareFoodPerception.asl "
                        + "-asl demos/foodOrdering/cookingActions.asl "
                        + "-asl demos/foodOrdering/humanActions.asl "
                        + "-asl demos/foodOrdering/ordering.asl "
                        + "-asl demos/foodOrdering/yumiInitPickAndPlace.asl "
                        + "-goal listen(self:agent) "
                        + "-goal init(self:agent) "
                        + "-editor ";

        createInstance(GoalManagerComponent.class, gmArgs);

        createInstance(SimSpeechRecognitionComponent.class, "-speaker eric -config yumiPickAndPlace.simspeech");
        createInstance(SimSpeechRecognitionComponent.class, "-speaker eric -config yumiPickAndPlaceJapanese.simspeech");
    }

    public static void main(String[] args) {
        PickAndPlaceLLMConfig demoConfig = new PickAndPlaceLLMConfig();
        demoConfig.runConfiguration();
    }
}
