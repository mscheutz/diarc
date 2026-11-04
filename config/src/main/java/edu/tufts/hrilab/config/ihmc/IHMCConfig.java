/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.ihmc;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.ihmc.IHMCComponent;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;

public class IHMCConfig extends DiarcConfiguration {

  private boolean useLLM = true;

  @Override
  public void runConfiguration() {

    // language pipeline components
    createInstance(ListenerComponent.class);
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);

    // nlu parser component(s)
    String hybridParserArgs = "-tldl spot.dict templatedict.dict -cache -cacheName spotPickAndPlace -cachePersist true";
    if (useLLM) {
      createInstance(LLMComponent.class);
      hybridParserArgs += " -llm -llmService spotPickAndPlace";
    } else {
      createInstance(SimpleNLGComponent.class);
    }

    createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, hybridParserArgs);

    // tts component
    createInstance(SimSpeechProductionComponent.class);
//    createInstance(MaryTTSComponent.class);
    //////////////////////////////////////////

    createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture nadia_ros2.xml -cameraFrame ros2_left_optical -baseFrame base_footprint");
//    createInstance(MockVisionComponent.class, "-sceneClass edu.tufts.hrilab.vision.scene.config.MultiRobotCaddySceneGenerator");

    createInstance(IHMCComponent.class, "-hubContainer 192.168.0.251@10001");

    createInstance(GoalManagerComponent.class,
            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                    "-beliefinitfile agents/agents.pl "
                    + "-beliefinitfile demos.pl "
                    + "-beliefinitfile demos/ihmc.pl "
                    + "-asl core.asl "
                    + "-asl dialogue/nlg.asl "
                    + "-asl dialogue/nlu.asl "
                    + "-asl dialogue/handleSemantics.asl "
                    + "-asl vision.asl "
                    + "-asl demos/ihmc.asl "
                    + "-goal listen(self:agent) "
                    + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
    );

  }
}
