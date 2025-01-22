/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.movebase.MoveBaseComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.tf.TFComponent;
import edu.tufts.hrilab.trade.gui.TRADEServiceGui;
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
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;


public class AutonomousUnityDIARCSpaceStationConfig extends DiarcConfiguration {
  private String llmEndpoint = "http://192.168.0.177:9900"; //"http://127.0.0.1:8080";
  private String agentName = "robot1";
  private int players = 1;
  private String goalManagerArgs;
  private boolean startAutonomy = true;
  private boolean adaptiveAutonomy = true;
  boolean mock = false;
  String unityPort = "1755";
  String unityIP = "127.0.0.1";
  String nullAddressee = "";

  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("llm").hasArg().argName("endpoint").desc("Set LLM endpoint.").build());
    options.add(Option.builder("agent").hasArg().argName("name").desc("Set robot agent name.").build());
    options.add(Option.builder("players").hasArg().argName("number").desc("Set number of human players (1 or 2).").build());
    options.add(Option.builder("unity").hasArg().argName("ip").desc("Unity IP.").build());
    options.add(Option.builder("port").hasArg().argName("port_num").desc("Unity Port").build());
    options.add(Option.builder("nonadaptive").desc("Flag to enable non-adaptive autonomy.").build());
    options.add(Option.builder("adaptive").desc("Flag to enable adaptive autonomy.").build());
    options.add(Option.builder("nullAddressee").desc("Flag to use a null addressee for the PR2.").build());
    return options;
  }

  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("llm")) {
      llmEndpoint = cmdLine.getOptionValue("llm");
    } else {
      log.warn("No LLM endpoint provided, using default: " + llmEndpoint);
    }
    if (cmdLine.hasOption("agent")) {
      agentName = cmdLine.getOptionValue("agent");
    }
    if (cmdLine.hasOption("players")) {
      players = Integer.parseInt(cmdLine.getOptionValue("players"));
    }
    if (cmdLine.hasOption("nonadaptive")) {
      startAutonomy = true;
    }
    if (cmdLine.hasOption("adaptive")) {
      startAutonomy = true;
      adaptiveAutonomy = true;
    }
    if (cmdLine.hasOption("nullAddressee")) {
      nullAddressee = " -nullAddressee";
    }
    if (cmdLine.hasOption("unity")) {
      unityIP = cmdLine.getOptionValue("ip");
    }
    if (cmdLine.hasOption("port")) {
      unityPort = cmdLine.getOptionValue("port_num");
    }
  }

  @Override
  public void runConfiguration() {

    // Language
    createInstance(HybridParserComponent.class, "-tldl pr2Unity.dict -tldlNoUpdateAddressee -patternMatching -matcher SpaceStationPatternMatcher -parsingSequence pattern tldl");
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);
    createInstance(SimpleNLGComponent.class, "-modifierService spaceStationModifyNLG");
    createInstance(UnitySpaceStationNLGComponent.class);
    createInstance(ListenerComponent.class);

    createInstance(MapComponent.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");

    if (mock) {
      createInstance(MockUnitySpaceStationComponent.class, "-config mock_space_station_settings.json");
      createInstance(MockMoveBaseComponent.class, "-groups agent:" + agentName + " -simExecTime");
      createInstance(MockUnityPR2Component.class, "-groups agent:" + agentName);
      createInstance(SimSpeechProductionComponent.class, "-gui -groups agent:" + agentName);
      for (int i = 0; i < players; i++) {
        String humanAgent = "player" + (i + 1);
        createInstance(MockHumanAgent.class, "-agent " + humanAgent + " -groups " + humanAgent);
        createInstance(SimSpeechRecognitionComponent.class, "-speaker " + humanAgent + " -groups " + humanAgent + " -config space_station.simspeech");
      }
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
      for (int i = 0; i < players; i++) {
        DiarcComponent.createInstance(HumanAgentComponent.class, "-agent player" + String.valueOf(i + 1));
      }
    }


    goalManagerArgs = "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
            "-priorityFile default.json space_station_priority.json " +
            "-goal listen(self) " +
            "-goal initializeTrial(" + agentName + ") " +
            "-asl domains/unity/space_station.asl " +
            "-asl domains/unity/space_station_autonomy.asl " +
            "-asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl ";
    // TODO: why are different pl files needed here?
    if (startAutonomy) {
      goalManagerArgs += "-beliefinitfile unity/pr2UnityTeam_smm.pl ";
      if (adaptiveAutonomy) {
        goalManagerArgs += "-goal autonomy(" + agentName + ",true) ";
      } else {
        goalManagerArgs += "-goal autonomy(" + agentName + ",false) ";
      }
    } else {
      goalManagerArgs += "-beliefinitfile unity/pr2UnityTeam_smm_" + agentName + ".pl ";
    }
    createInstance(GoalManagerComponent.class, goalManagerArgs);


    // TODO: add guis to set workloads
//    createInstance(TRADEServiceGui.class, "-class edu.tufts.hrilab.unity.space_station.MockUnitySpaceStationComponent");
  }
}
