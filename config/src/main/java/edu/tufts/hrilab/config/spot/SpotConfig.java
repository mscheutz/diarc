/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.spot;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.vision.MockVisionComponent;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public class SpotConfig extends DiarcConfiguration {

  boolean useMockVision = true;
  String service = "llamahf";
  String model = "Meta-Llama-3.1-70B-Instruct";
  boolean useLLM = false; //Ask Eric for HF key if you want to run with the LLM

  public SpotConfig() {
  }

  @Override
  public void runConfiguration() {

    createInstance(ListenerComponent.class);
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);
    createInstance(SimSpeechProductionComponent.class);

    String hybridParserArgs = "-tldl spot.dict templatedict.dict -cache -cacheName spotPickAndPlace -cachePersist true";
    if (useLLM) {
      createInstance(LLMComponent.class);
      hybridParserArgs += " -llm -llmService spotPickAndPlace";
    } else {
      createInstance(SimpleNLGComponent.class);
    }

    createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, hybridParserArgs);
    if (useMockVision) {
      createInstance(MockVisionComponent.class, "-groups agent:robotone -sceneClass edu.tufts.hrilab.vision.scene.config.SpotSceneGenerator");
    } else {
//      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-groups agent:robotone -capture spot.xml -cameraFrame hand_color_image_sensor");
      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-groups agent:robotone -capture spot_ros2.xml -cameraFrame hand_color_image_sensor");
//      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture pcd.xml -height 1.1 -rotX -45");
    }

    String spotConfigFile = "/home/evan/Downloads/spot_phi.yaml";
    String tradeHubContainer = "127.0.0.1@10002";
//    String tradeHubContainer = "192.168.0.251@10001";
    createInstance(SpotComponent.class, "-spotConfigFile " + spotConfigFile + " -groups agent:robotone -hubContainer " + tradeHubContainer);

    // flex rl
//    PythonWrapper flex = new PythonWrapper("diarc.flex.flex_wrapper", true, tradeHubContainer);
//    flex.start("--groups", "agent:robotone"); // Start the wrapper

    createInstance(GoalManagerComponent.class,
            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                    "-beliefinitfile agents/agents.pl "
                    + "-beliefinitfile demos.pl "
                    + "-beliefinitfile demos/spotVA.pl "
                    + "-asl core.asl "
                    + "-asl dialogue/nlg.asl "
                    + "-asl dialogue/nlu.asl "
                    + "-asl dialogue/handleSemantics.asl "
                    + "-asl vision.asl "
//                    + "-asl demos/spotDemo.asl "
//                    + "-asl spot/spotActions.asl "
                    + "-goal listen(self) "
//                    + "-goal init(self) "
                    + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
    );

    createInstance(SimSpeechRecognitionComponent.class, "-speaker evan -config spotDemo.simspeech");

    Util.Sleep(2000);

    try {
      TRADEServiceInfo sitService = TRADE.getAvailableService(new TRADEServiceConstraints().name("sit").argTypes());
      TRADEServiceInfo testService_1 = TRADE.getAvailableService(new TRADEServiceConstraints().name("testMethod").argTypes());
      TRADEServiceInfo testService_2 = TRADE.getAvailableService(new TRADEServiceConstraints().name("testMethod").argTypes(String.class));
      TRADEServiceInfo moveToService_1 = TRADE.getAvailableService(new TRADEServiceConstraints().name("moveTo").argTypes(String.class, Symbol.class));
      TRADEServiceInfo moveToService_2 = TRADE.getAvailableService(new TRADEServiceConstraints().name("moveTo").argTypes(String.class, Point3d.class, Quat4d.class));

      sitService.call(Justification.class);
      testService_1.call(Justification.class);
      testService_2.call(Justification.class, "string");
      moveToService_1.call(Justification.class, "string", Factory.createSymbol("symbol"));
      moveToService_2.call(Justification.class, "string", new Point3d(), new Quat4d());
    } catch (TRADEException e) {
      throw new RuntimeException(e);
    }

  }

}
