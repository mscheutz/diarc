/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.movebase.MockMoveBaseComponent;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;
import edu.tufts.hrilab.slug.parsing.llm.LLMParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.unity.*;
import edu.tufts.hrilab.unity.space_station.MockHumanAgent;
import edu.tufts.hrilab.unity.space_station.MockUnitySpaceStationComponent;
import edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM;
import edu.tufts.hrilab.unity.space_station.nlg.UnitySpaceStationNLGComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class MockUnityDIARCSpaceStationConfig extends DiarcConfiguration {
  private String llmEndpoint = "http://192.168.0.177:9900"; //"http://127.0.0.1:8080";
  private String agentName = "robot1";
  private String disableAutonomy = "";
  private int players = 1;
  private String goalManagerArgs;

  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("llm").hasArg().argName("endpoint").desc("Set LLM endpoint.").build());
    options.add(Option.builder("agent").hasArg().argName("name").desc("Set robot agent name.").build());
    options.add(Option.builder("players").hasArg().argName("number").desc("Set number of human players (1 or 2).").build());
    options.add(Option.builder("autonomy").desc("Flag to enable autonomy.").build());
    options.add(Option.builder("disableAutonomyFirst").desc("Flag to disable autonomy at first.").build());
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

    if (cmdLine.hasOption("disableAutonomyFirst")) {
      disableAutonomy = " -goal disableAutonomyFirst(" + agentName + ":agent)";
    }

    if (cmdLine.hasOption("autonomy")) {
      goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(" + agentName + ":agent) -goal startAutonomy(" + agentName + ":agent) -beliefinitfile unity/pr2UnityTeam_smm.pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl domains/unity/space_station_llm_llama_finetuned_autonomy.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl" + disableAutonomy;
    } else {
      goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(" + agentName + ":agent) -beliefinitfile unity/pr2UnityTeam_smm_" + agentName + ".pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    }
  }

  @Override
  public void runConfiguration() {
    // mock sim env
    createInstance(MockUnitySpaceStationComponent.class, "-config mock_space_station_settings.json");

//    createInstance(LLMParserComponent.class, "-service spaceStationLLMParser");
    createInstance(HybridParserComponent.class, "-tldl pr2Unity.dict  -tldlNoUpdateAddressee -llm -llmService spaceStationLLMParser -noConfirmation -cache -cacheName unity_space_station");
    createInstance(LLMComponent.class, "-groups agent:" + agentName + " -service llama -model 7B -endpoint " + llmEndpoint + " -stopwords }");
    createInstance(UnitySpaceStationLLM.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");

    createInstance(ListenerComponent.class);
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);
    createInstance(SimpleNLGComponent.class, "-modifierService spaceStationModifyNLG");
    createInstance(UnitySpaceStationNLGComponent.class);

    createInstance(MapComponent.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");
    createInstance(MockMoveBaseComponent.class, "-groups agent:" + agentName);
    createInstance(MockUnityPR2Component.class, "-groups agent:"  + agentName);
    createInstance(SimSpeechProductionComponent.class, "-gui -groups agent:"  + agentName);
//    createInstance(MockUnityPR2Component.class, "-groups agent:"  + agentName2); // add for two robot case

    createInstance(GoalManagerComponent.class, goalManagerArgs);

    for (int i = 0; i < players; i++) {
      String humanAgent = "player" + (i + 1);
      createInstance(MockHumanAgent.class, "-agent " + humanAgent + " -groups " + humanAgent);
      createInstance(SimSpeechRecognitionComponent.class, "-speaker " + humanAgent + " -groups " + humanAgent + " -config space_station.simspeech");
    }

    // TODO: add guis to set workloads
  }
}
