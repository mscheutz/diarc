/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.movebase.MoveBaseComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.tf.TFComponent;
import edu.tufts.hrilab.unity.MockUnityPR2Component;
import edu.tufts.hrilab.unity.UnityAgentComponent;
import edu.tufts.hrilab.unity.UnityComponent;
import edu.tufts.hrilab.unity.UnityPR2Component;
import edu.tufts.hrilab.unity.space_station.HumanAgentComponent;
import edu.tufts.hrilab.unity.space_station.MockHumanAgent;
import edu.tufts.hrilab.unity.space_station.MockUnitySpaceStationComponent;
import edu.tufts.hrilab.unity.space_station.UnitySpaceStationComponent;
import edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM;
import edu.tufts.hrilab.unity.space_station.nlg.UnitySpaceStationNLGComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.List;


public class AutonomousUnityDIARCSpaceStationConfig {
  protected static Logger log = LoggerFactory.getLogger(AutonomousUnityDIARCSpaceStationConfig.class);

  public static void main(String[] args) {
    List<String> largs = Arrays.asList(args);
    String unityIP = "127.0.0.1";
    String llmEndpoint = "http://127.0.0.1:8080";
    String unityPort = "1755";
    String agentName = "robot1";
    String nullAddressee = "";
    int players = 1;
    boolean mock = false;
    boolean startAutonomy = true;
    boolean adaptiveAutonomy = true;

    if (largs.contains("-unity")) {
      unityIP = largs.get(largs.indexOf("-unity") + 1);
    } else {
      log.warn("No unity IP provided, using default: " + unityIP);
    }

    if (largs.contains("-port")) {
      unityPort = largs.get(largs.indexOf("-port") + 1);
    } else {
      log.warn("No unity port provided, using default: " + unityPort);
    }

    if (largs.contains("-llm")) {
      llmEndpoint = largs.get(largs.indexOf("-llm") + 1);
    } else {
      log.warn("No LLM endpoint provided, using default: " + llmEndpoint);
    }

    if (largs.contains("-agent")) {
      agentName = largs.get(largs.indexOf("-agent") + 1);
    }

    if (largs.contains("-nullAddressee")) {
      nullAddressee = " -nullAddressee";
    }

    if (largs.contains("-players")) {
      players = Integer.parseInt(largs.get(largs.indexOf("-players") + 1));
    }

    // Language
    DiarcComponent.createInstance(HybridParserComponent.class, "-tldl pr2Unity.dict -tldlNoUpdateAddressee -patternMatching -matcher SpaceStationPatternMatcher -parsingSequence pattern tldl");
    DiarcComponent.createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    DiarcComponent.createInstance(ReferenceResolutionComponent.class, "");
    DiarcComponent.createInstance(DialogueComponent.class, "");
//    DiarcComponent.createInstance(SimpleNLGComponent.class, "-modifierService spaceStationModifyNLG");
    DiarcComponent.createInstance(HybridNLGComponent.class, "-cacheLoad ss_nlg_cache.sqlite -cachePersist");
    DiarcComponent.createInstance(UnitySpaceStationNLGComponent.class, "");
    DiarcComponent.createInstance(ListenerComponent.class, "");

    DiarcComponent.createInstance(MapComponent.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");

    if (mock) {
      DiarcComponent.createInstance(MockUnitySpaceStationComponent.class, "-config mock_space_station_settings.json");
      DiarcComponent.createInstance(MockMoveBaseComponent.class, "-groups agent:" + agentName + " -simExecTime");
      DiarcComponent.createInstance(MockUnityPR2Component.class, "-groups agent:" + agentName);
      DiarcComponent.createInstance(SimSpeechProductionComponent.class, "-gui -groups agent:" + agentName);
    } else {
      // Unity
      DiarcComponent.createInstance(UnitySpaceStationComponent.class, "-agent spacestation -groups agent:" + agentName);
      DiarcComponent.createInstance(UnityPR2Component.class, "-agent " + agentName + " -groups agent:" + agentName + nullAddressee);
      DiarcComponent.createInstance(UnityComponent.class, "-unityip " + unityIP + " -unityport " + unityPort + " -groups agent:" + agentName);
      DiarcComponent.createInstance(UnitySpaceStationLLM.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");
      DiarcComponent.createInstance(UnityAgentComponent.class, "-agent rover -groups agent:" + agentName);
      //PR2
      DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:" + agentName + " -point_dist_thresh 0.5");
      DiarcComponent.createInstance(TFComponent.class, " -groups agent:" + agentName);
    }

    String goalManagerArgs = "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
            "-priorityFile default.json space_station_priority.json " +
            "-goal listen(self) " +
            "-asl domains/unity/space_station.asl " +
            "-asl domains/unity/space_station_autonomy.asl " +
            "-asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl ";
    // TODO: why are different pl files needed here?
    if (startAutonomy) {
      // set init autonomy area to monitor
      String initArea;
      if (agentName.equals("robot1")) {
        initArea = "alpha";
      } else {
        initArea = "beta";
      }
      goalManagerArgs += "-goal initializeTrial(" + agentName + "," + initArea + "," + adaptiveAutonomy + ") ";
      goalManagerArgs += "-beliefinitfile unity/pr2UnityTeam_smm.pl ";
    } else {
      goalManagerArgs += "-beliefinitfile unity/pr2UnityTeam_smm_" + agentName + ".pl ";
    }
    DiarcComponent.createInstance(GoalManagerComponent.class, goalManagerArgs);


    // start human component *after* GM
    if (mock) {
      for (int i = 0; i < players; i++) {
        String humanAgent = "player" + (i + 1);
        DiarcComponent.createInstance(MockHumanAgent.class, "-agent " + humanAgent + " -groups " + humanAgent);
        DiarcComponent.createInstance(SimSpeechRecognitionComponent.class, "-speaker " + humanAgent + " -groups " + humanAgent + " -config space_station.simspeech");
      }
    } else {
      for (int i = 0; i < players; i++) {
        DiarcComponent.createInstance(HumanAgentComponent.class, "-agent player" + (i + 1));
      }
    }
  }
}