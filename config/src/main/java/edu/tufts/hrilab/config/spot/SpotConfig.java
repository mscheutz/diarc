/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.spot;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.ftrontend.FrontEndConfigurationComponent;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.llm.serviceproviders.tr.social.SocialSpotLLMServiceProvider;
import edu.tufts.hrilab.llm.serviceproviders.tr.social.SocialTRLLMServiceProvider;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.spot.MockSpotComponent;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.teleop.TeleopGui;
import edu.tufts.hrilab.trade.gui.TRADEServiceGui;
import edu.tufts.hrilab.vision.MockVisionComponent;

import javax.swing.*;
public class SpotConfig extends DiarcConfiguration {

  boolean useMockVision = false;
  boolean useMockSpot = false;
  String service = "openai";
  String model = "QuantTrio/Qwen3.5-9B-AWQ";
  String openaiEndpoint = "https://dashboard.thinkingrobots.ai/qwen";
  boolean useLLM = true; //Ask Eric for OpenAI/HF key if you want to run with the LLM

  public SpotConfig() {
  }

  @Override
  public void runConfiguration() {

    createInstance(ListenerComponent.class);
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);

//    if (useMockVision) {
//      createInstance(MockVisionComponent.class, "-sceneClass edu.tufts.hrilab.vision.scene.config.SpotSceneGenerator");
//    } else {
//      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture spot.xml -cameraFrame hand_color_image_sensor");
      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture spot_python.xml -baseFrame gpe -cameraFrame hand_color_image_sensor");
//      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture pcd.xml -height 1.1 -rotX -45");
//    }

//    if (useMockSpot) {
//      createInstance(MockSpotComponent.class, "-refs mockSpotLocationRefs.json");
//    } else {
//    String spotConfigFile = "/home/hrilab/Downloads/spot_psi.yaml";
    String spotConfigFile = "/home/evan/Documents/spot/spot_psi.yaml";
//      String spotConfigFile = "/home/evan/Documents/spot/spot_phi.yaml";
//    String tradeHubContainer = "127.0.0.1@10001";
      String tradeHubContainer = "192.168.0.46@10001";
      createInstance(SpotComponent.class, "-spotConfigFile " + spotConfigFile
              + " -hubContainer " + tradeHubContainer
              + " -spotMap " + "/home/evan/.diarc/spot/spot_map"
              + " -refs " + "/spot/spot_map_refs.json"
      );
//    }

    createInstance(GoalManagerComponent.class,
            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                    "-beliefinitfile agents/agents.pl "
                    + "-beliefinitfile demos.pl "
                    + "-beliefinitfile demos/spotVA.pl "
                    + "-asl core.asl "
                    + "-asl dialogue/nlg.asl "
                    + "-asl dialogue/nlu.asl "
                    + "-asl dialogue/socialLLMnlu.asl "
                    + "-asl dialogue/handleSemantics.asl "
                    + "-asl vision.asl "
//                    + "-asl demos/spotDemo.asl "
                    + "-asl demos/spotRoomClearing.asl "
//                    + "-asl spot/spotActions.asl "
                    + "-asl spot/spot.asl "
                    + "-asl demos/queueEM.asl "
                    + "-asl ava/socialAvaIntrospection.asl "
                    + "-goal listen(self) "
                    + "-goal initRooms(self) "
                    + "-priorityFile default.json avaPriority.json "
    );

    //TODO: proper configuration (ww needs to be off)
    createInstance(FrontEndConfigurationComponent.class,"");
    String hybridParserArgs = "-tldl spot.dict templatedict.dict -cache -cacheName spotPickAndPlace -cachePersist true";
    String hybridNlgArgs = "-simpleNLG -cache -cacheName ava-reverse -cachePersist";
    if (useLLM) {
      createInstance(SocialSpotLLMServiceProvider.class,"");
      String llmComponentArgs = String.format("-service %s hf -model %s Qwen/Qwen3.5-9B -temperature 0.0 0.0 -endpoint %s", service, model, openaiEndpoint);
      createInstance(edu.tufts.hrilab.llm.LLMComponent.class, llmComponentArgs);
      hybridParserArgs += " -llm -sequence llm cache tldl -noConfirmation";
      hybridNlgArgs += " -llm -sequence llm cache simple";
    }
    createInstance(edu.tufts.hrilab.slug.parsing.hybrid.domains.AvaHybridParserComponent.class, hybridParserArgs);
    createInstance(edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent.class, hybridNlgArgs);

    createInstance(SimSpeechRecognitionComponent.class, "-speaker local_user -addressee spot -config spotRoomClearing.simspeech");

    boolean muteMics = true;//false;
    String speakerId = "";
    boolean useCartesia = false;
    String cartesiaVoiceId = "";
    boolean useLocalTTS = true;
    String localVoice = "";
    String ttsEndpoint = "";
    String voiceVolume = "1.15";//"1.0";
    String voiceSpeed = "fast";//"normal";
    String voiceClipVolume = "0.4";
    if (useLocalTTS) {
      String localArgs = " -volume " + voiceVolume + " -speed " + voiceSpeed + " -clipVolume " + voiceClipVolume;
      if (!speakerId.isEmpty()) {
        localArgs += " -matchMixer " + speakerId;
      }
      if (!localVoice.isEmpty()) {
        localArgs += " -voiceId " + localVoice;
      }
      if (!ttsEndpoint.isEmpty()) {
        localArgs += " -ttsEndpoint " + ttsEndpoint;
      }
      if (muteMics) {
        localArgs += " -muteMics";
      }
      createInstance(edu.tufts.hrilab.ava.tts.AvaLocalTTSComponent.class, localArgs);
    } else if (useCartesia) {
      String cartesiaArgs = " -volume " + voiceVolume + " -speed " + voiceSpeed + " -clipVolume " + voiceClipVolume;
      if (!speakerId.isEmpty()) {
        cartesiaArgs += " -matchMixer " + speakerId;
      }
      if (!cartesiaVoiceId.isEmpty()) {
        cartesiaArgs += " -voiceId " + cartesiaVoiceId;
      }
      if (muteMics) {
        cartesiaArgs += " -muteMics";
      }
      createInstance(edu.tufts.hrilab.ava.tts.AvaCartesiaTTSComponent.class, cartesiaArgs);
    } else {
      String maryTTSArgs = " -female -volume 1.0 -showMixers";
      if (!speakerId.isEmpty()) {
        maryTTSArgs += " -matchMixer " + speakerId;
      }
      createInstance(edu.tufts.hrilab.ava.tts.AvaMaryTTSComponent.class, maryTTSArgs);
    }

    SwingUtilities.invokeLater(() -> new TeleopGui().start(new TeleopGui.TRADEVelocityPublisher()));

    createInstance(TRADEServiceGui.class, "-class edu.tufts.hrilab.spot.SpotInterface");

  }

}
