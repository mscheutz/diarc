package edu.tufts.hrilab.config.llm;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.abb.MockYumiComponent;
import edu.tufts.hrilab.abb.PickAndPlaceLLM;
import edu.tufts.hrilab.abb.RequestFromHumanComponent;
import edu.tufts.hrilab.abb.consultant.area.ABBAreaConsultant;
import edu.tufts.hrilab.abb.consultant.cognex.CognexConsultant;
import edu.tufts.hrilab.abb.consultant.item.ItemConsultant;
import edu.tufts.hrilab.abb.consultant.location.ABBLocationConsultant;
import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;

public class PickAndPlaceLLMDemo extends DiarcConfiguration {
    protected static Logger log = LoggerFactory.getLogger(PickAndPlaceLLMDemo.class);
    public SimSpeechRecognitionComponent simSpeechRec;
    public MockYumiComponent leftArm;
    public MockYumiComponent rightArm;
    private CognexConsultant cognexConsultant;
    private ItemConsultant itemConsultant;
    private ABBAreaConsultant areaConsultant;
    private ABBLocationConsultant locationConsultant;
    String service = "llamahf";
    String model = "Meta-Llama-3-70B-Instruct";

    @Override
    public void runConfiguration() {
        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);

        leftArm = createInstance(MockYumiComponent.class, "-groups agent:leftArm:yumi");
        rightArm = createInstance(MockYumiComponent.class, "-groups agent:rightArm:yumi");

        cognexConsultant = new CognexConsultant();
        try {
            TRADE.registerAllServices(cognexConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "agent:human:mobileManipulator", "physobj")));
        } catch (TRADEException e) {
            log.error("error registering cognex consultant in DIARC config", e);
        }
        leftArm.setCognexConsultant(cognexConsultant);
        rightArm.setCognexConsultant(cognexConsultant);

        createInstance(RequestFromHumanComponent.class, "-groups agent:human:mobileManipulator");

        itemConsultant = new ItemConsultant();
        try {
            TRADE.registerAllServices(itemConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "item")));
        } catch (TRADEException e) {
            log.error("error registering item consultant in DIARC config", e);
        }

        areaConsultant = new ABBAreaConsultant();
        try {
            TRADE.registerAllServices(areaConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "area")));
        } catch (TRADEException e) {
            log.error("error registering area consultant in DIARC config", e);
        }

        locationConsultant = new ABBLocationConsultant();
        try {
            TRADE.registerAllServices(locationConsultant, new ArrayList<>(Arrays.asList("agent:leftArm:yumi", "agent:rightArm:yumi", "agent:self:agent", "agent:human:mobileManipulator", "location")));
        } catch (TRADEException e) {
            log.error("error registering location consultant in DIARC config" + e);
        }

        createInstance(LLMComponent.class, String.format("-service %s -model %s -temperature 0.01", service, model));
        createInstance(PickAndPlaceLLM.class, "-nluPrompt pickAndPlace/nlu/pickAndPlaceActionSemanticTranslationYumi -outputLanguage ja");
        createInstance(edu.tufts.hrilab.slug.parsing.llm.LLMParserComponent.class, "-service pickAndPlaceLLMParser");

        createInstance(ReferenceResolutionComponent.class);
        createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
        createInstance(DialogueComponent.class, "");

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
                        + "-goal init(self:agent) ";

        createInstance(GoalManagerImpl.class, gmArgs);

        simSpeechRec = createInstance(SimSpeechRecognitionComponent.class, "-speaker eric -config yumiPickAndPlace.simspeech -nogui");
    }

    @Override
    public void shutdownConfiguration() {
        super.shutdownConfiguration();
        try {
            TRADE.deregister(areaConsultant);
            TRADE.deregister(itemConsultant);
            TRADE.deregister(cognexConsultant);
            TRADE.deregister(locationConsultant);
        } catch (TRADEException e) {
            log.error("[shutdownConfiguration] Error deregistering consultants", e);
        }
    }

    public static void main(String[] args) {
        PickAndPlaceLLMDemo demoConfig = new PickAndPlaceLLMDemo();
        demoConfig.runConfiguration();
    }
}
