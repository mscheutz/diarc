/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.kinova;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.moveit.MockMoveItComponent;
import edu.tufts.hrilab.moveit.MoveItComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.tf.MockTFComponent;
import edu.tufts.hrilab.tf.TFComponent;
import edu.tufts.hrilab.trade.gui.TRADEServiceGui;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.vision.stm.Grasp;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class KinovaFlexConfig extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(KinovaFlexConfig.class);

  protected boolean useLLM = false;
  protected boolean useMockRobot = false;

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

    createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-cameraFrame camera_color_frame -capture kinova.xml");
//     createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture pcd.xml");

//    createInstance(edu.tufts.hrilab.grasp.GraspComponent.class);

    MoveItComponent moveItComponent = null;
    if (useMockRobot) {
      createInstance(MockTFComponent.class);
      createInstance(MockMoveItComponent.class);
    } else {
      createInstance(TFComponent.class);
      moveItComponent = createInstance(MoveItComponent.class, "-config kortex.json");
    }

    // flex rl
//    String tradeHubContainer = "192.168.80.101@10001";
    String tradeHubContainer = "127.0.0.1@10001";
    PythonWrapper flex = new PythonWrapper("diarc.flex.flex_wrapper", true, tradeHubContainer);
    flex.start(); // Start the wrapper

    createInstance(GoalManagerComponent.class,
            "-beliefinitfile agents/agents.pl "
                    + "-beliefinitfile demos.pl "
                    + "-asl core.asl "
                    + "-asl dialogue/nlg.asl "
                    + "-asl dialogue/nlu.asl "
                    + "-asl dialogue/handleSemantics.asl "
                    + "-asl vision.asl "
                    + "-asl manipulation.asl "
                    + "-goal listen(self) ");


    createInstance(SimSpeechRecognitionComponent.class, "-speaker evan -addressee self -config spotDemo.simspeech");

    createInstance(TRADEServiceGui.class, "-class edu.tufts.hrilab.moveit.MoveItComponent");

//    moveItComponent.goToPose("arm", Factory.createSymbol("home"));
  }
}
