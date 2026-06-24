/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.spot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.Instant;
import java.util.ArrayList;

import javax.swing.SwingUtilities;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.spot.MockSpotComponent;
import edu.tufts.hrilab.spot.SpotComponent;
import edu.tufts.hrilab.teleop.TeleopGui;
import edu.tufts.hrilab.vision.MockVisionComponent;

public class SpotLLMConfig extends DiarcConfiguration {

  boolean useMockVision = true;
  boolean useMockSpot = true;
  String service = "llamahf";
  String model = "Meta-Llama-3.1-70B-Instruct";
  boolean useLLM = false; //Ask Eric for HF key if you want to run with the LLM

  boolean writeSpeechToFile = false;
  String speechFile = System.getProperty("user.home") + "/Desktop/dialogues/dialogue_history_" + Instant.now().toString() + ".txt";

  public SpotLLMConfig() {
  }

  @Override
  public void runConfiguration() {

    createInstance(ListenerComponent.class);
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(edu.tufts.hrilab.llm.LLMComponent.class, "-service openai");
    

    
    if (useMockVision) {
      createInstance(MockVisionComponent.class, "-sceneClass edu.tufts.hrilab.vision.scene.config.SpotSceneGenerator");
    } else {
      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture spot.xml -cameraFrame hand_color_image_sensor");
      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture spot_python.xml -baseFrame gpe -cameraFrame hand_color_image_sensor");
      createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture pcd.xml -height 1.1 -rotX -45");
    }

    if (useMockSpot) {
      createInstance(MockSpotComponent.class);
    } else {
      String spotConfigFile = "/home/evan/Documents/spot/spot_psi.yaml";

      String tradeHubContainer = "192.168.0.46@10001";
      createInstance(SpotComponent.class, "-spotConfigFile " + spotConfigFile
              + " -hub " + tradeHubContainer
              + " -spotMap " + "/home/evan/.diarc/spot/spot_map"
              + " -refs " + "/spot/spot_map_refs.json");
    }

    createInstance(GoalManagerComponent.class,
            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " 
                    + "-beliefinitfile spot.pl "
                    + "-beliefinitfile demos/spotVA.pl "
                    + "-beliefinitfile demos.pl "
                    + "-asl core.asl "
                    + "-asl vision.asl "
//                    + "-asl spot/spotActions.asl "
                    + "-asl demos/spotVisionDesc.asl "
                    + "-asl spot/spot.asl "
                    + "-asl spot/spotLLM.asl "
                    + "-goal bring_me_my_cup(self) "
//                    + "-goal init(self) "
                    + "-selector edu.tufts.hrilab.action.selector.LLMActionSelector "
    );

    //createInstance(SimSpeechRecognitionComponent.class, "-speaker evan -addressee self -config spotVisionDesc.simspeech");

    SwingUtilities.invokeLater(() -> new TeleopGui().start(new TeleopGui.TRADEVelocityPublisher()));

    /////////////////////////////////////////////
    // write speech input and output to file

    if (writeSpeechToFile) {
      File file = new File(speechFile);
      if (!file.exists()) {
        try {
          File parent = file.getParentFile();
          parent.mkdirs();
          file.createNewFile();
        } catch (IOException e) {
          log.error("Could not create dialogue history file.", e);
        }
      }

      // register writeSpeechToFile service
      try {
        TRADE.registerAllServices(this, new ArrayList<>());
      } catch (TRADEException e) {
        log.error("Error registering services with TRADE", e);
      }

      try {
        TRADEServiceInfo wrapperTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("writeSpeechToFile").argTypes(Object[].class));
        TRADEServiceInfo sayTextTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(String.class));
        TRADEServiceInfo reportRecognizedSpeechTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class));
        TRADE.afterService(wrapperTSI, sayTextTSI);
        TRADE.afterService(wrapperTSI, reportRecognizedSpeechTSI);
      } catch (TRADEException e) {
        log.error("[cancelTaskTest] add join on goal wrapper", e);
      }
    }

  }
  @TRADEService
  public void writeSpeechToFile(Object[] args) {
    String service = (String) args[0];
    Object arg = args[1];
    try {
      FileWriter fw = new FileWriter(speechFile, true);
      if (service.equals("sayText")) {
        // from sayText
        fw.write("robot: " + arg.toString() + "\n");
      } else {
        // from reportRecognizedSpeech
        Utterance utterance = (Utterance) arg;
        fw.write("person: " + utterance.getWordsAsString() + "\n");
      }
      fw.close();
    } catch (IOException e) {
      log.error("Failed to write dialogue history to file.", e);
    }
  }

}
