/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.spot;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.flex.FlexComponent;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.vision.MockVisionComponent;

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
    createInstance(SpotComponent.class, "-spotConfigFile " + spotConfigFile + " -groups agent:robotone -tradeHub " + tradeHubContainer);

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
                    + "-asl demos/spotDemo.asl "
                    + "-asl spot/spotActions.asl "
                    + "-goal listen(self) "
//                    + "-goal init(self) "
                    + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
    );

    createInstance(SimSpeechRecognitionComponent.class, "-speaker evan -config spotDemo.simspeech");

  }

}
