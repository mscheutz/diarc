/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.abb.RequestFromHumanComponent;
import edu.tufts.hrilab.abb.MockYumiComponent;
import edu.tufts.hrilab.abb.consultant.area.ABBAreaConsultant;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.cognex.consultant.CognexConsultant;
import edu.tufts.hrilab.abb.consultant.item.ItemConsultant;
import edu.tufts.hrilab.abb.consultant.location.ABBLocationConsultant;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;

public class YumiFoodOrderingMock extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(YumiFoodOrderingMock.class);
  static public SimSpeechRecognitionComponent simSpeechRec;
  static public SimSpeechRecognitionComponent untrustedSimSpeechRec;
  public MockYumiComponent leftArm;
  public MockYumiComponent rightArm;
  private CognexConsultant cognexConsultant;
  private ItemConsultant itemConsultant;
  private ABBAreaConsultant areaConsultant;
  private ABBLocationConsultant locationConsultant;
  private boolean firebase;
  private boolean test;
  public GoalManagerComponent gm;

  public YumiFoodOrderingMock(boolean test) {
    this.test = test;
  }

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

    createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, "-tldl foodOrdering.dict -tldl yumiFoodOrderingHomophones.dict -tldlNoUpdateAddressee -patternMatching");

    createInstance(ReferenceResolutionComponent.class);
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(DialogueComponent.class, "");
    createInstance(SimpleNLGComponent.class, "");

    createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");

    String gmArgs = "";
    if (!test) {
      gmArgs += "-editor -beliefg ";
    }
    gmArgs +=
            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager "
                    + "-diarcagent leftArm:yumi right:yumi human:mobileManipulator "
                    + "-beliefinitfile demos.pl "
                    + "-beliefinitfile agents/yumiFoodOrderingAgents.pl "
                    + "-beliefinitfile demos/yumiFoodOrdering.pl "
                    + "-beliefinitfile demos/yumiFoodOrderingHardcodedFacts.pl "
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
                    + "-asl demos/foodOrdering/yumiInit.asl "
                    + "-goal listen(self:agent)";
//                        + "-goal init(self:agent)";

    gm = createInstance(GoalManagerComponent.class, gmArgs);

    if (test) {
      untrustedSimSpeechRec = createInstance(SimSpeechRecognitionComponent.class, "-speaker front -config yumiFoodOrdering.simspeech -nogui");
      simSpeechRec = createInstance(SimSpeechRecognitionComponent.class, "-speaker brad -config yumiFoodOrdering.simspeech -nogui");
    } else {
      createInstance(SimSpeechRecognitionComponent.class, "-speaker front -config yumiFoodOrdering.simspeech");
      createInstance(SimSpeechRecognitionComponent.class, "-speaker brad -config yumiFoodOrdering.simspeech");
    }
  }

  @Override
  public void shutdownConfiguration() {
    try {
      TRADE.deregister(areaConsultant);
      TRADE.deregister(itemConsultant);
      TRADE.deregister(cognexConsultant);
      TRADE.deregister(locationConsultant);
    } catch (TRADEException e) {
      log.error("[shutdownConfiguration] Error deregistering consultants", e);
    }
    super.shutdownConfiguration();
  }

  public static void main(String[] args) {
    boolean test = false;
    YumiFoodOrderingMock demoConfig = new YumiFoodOrderingMock(test);
    demoConfig.runConfiguration();
    //SpringApplication.run(GuiManager.class, args);
  }
}