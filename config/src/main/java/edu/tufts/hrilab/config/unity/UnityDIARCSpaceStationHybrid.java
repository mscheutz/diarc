/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.diarc.DiarcComponent;

import edu.tufts.hrilab.movebase.MoveBaseComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.tf.TFComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.unity.UnityComponent;
import edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM;
import edu.tufts.hrilab.unity.space_station.nlg.UnitySpaceStationNLGComponent;
import edu.tufts.hrilab.unity.space_station.UnitySpaceStation;
import edu.tufts.hrilab.unity.space_station.HumanAgent;
import edu.tufts.hrilab.unity.UnityAgent;
import edu.tufts.hrilab.unity.UnityPR2;
import edu.tufts.hrilab.action.GoalManagerComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.Arrays;
import java.util.Dictionary;
import java.util.List;


public class UnityDIARCSpaceStationHybrid {
  // for logging
  protected static Logger log = LoggerFactory.getLogger(UnityDIARCSpaceStationHybrid.class);

  public static void main(String[] args) {
    List<String> largs = Arrays.asList(args);
    String unityIP = "127.0.0.1";
    String llmEndpoint = "http://127.0.0.1:8080";
    String unityPort = "1755";
    String agentName = "robot1";
    String nullAddressee = "";
    int players = 1;

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



    String goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(" + agentName + ":agent) -beliefinitfile unity/pr2UnityTeam_smm_" + agentName + ".pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    if (largs.contains("-autonomy")) {
      goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(" + agentName + ":agent) -goal startAutonomy(" + agentName + ":agent) -beliefinitfile unity/pr2UnityTeam_smm.pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl domains/unity/space_station_llm_llama_finetuned_autonomy.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    }

    DiarcComponent.createInstance(HybridParserComponent.class, "-tldl pr2Unity.dict  -tldlNoUpdateAddressee -llm -llmService spaceStationLLMParser -noConfirmation -cache -cacheName unity_space_station"); //

    DiarcComponent.createInstance(MapComponent.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:" + agentName + " -point_dist_thresh 0.5");
    DiarcComponent.createInstance(TFComponent.class, " -groups agent:"  + agentName);

    DiarcComponent.createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    DiarcComponent.createInstance(ReferenceResolutionComponent.class, "");
    DiarcComponent.createInstance(DialogueComponent.class, "");
    DiarcComponent.createInstance(SimpleNLGComponent.class, "-modifierService spaceStationModifyNLG");
    DiarcComponent.createInstance(UnitySpaceStationNLGComponent.class, "");
    DiarcComponent.createInstance(ListenerComponent.class, "");

    DiarcComponent.createInstance(LLMComponent.class, "-groups agent:" + agentName + " -service llama -model 7B -endpoint " + llmEndpoint + " -stopwords }");

    DiarcComponent.createInstance(UnityComponent.class, "-unityip " + unityIP  + " -unityport " + unityPort + " -groups agent:"  + agentName);
    DiarcComponent.createInstance(UnitySpaceStation.class, "-agent spacestation -groups agent:"  + agentName);
    DiarcComponent.createInstance(UnitySpaceStationLLM.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(UnityAgent.class, "-agent rover -groups agent:" + agentName);
    DiarcComponent.createInstance(UnityPR2.class, "-agent " + agentName + " -groups agent:"  + agentName + nullAddressee );

    DiarcComponent.createInstance(GoalManagerComponent.class, goalManagerArgs);

    for (int i = 0; i < players; i++) {
      DiarcComponent.createInstance(HumanAgent.class, "-agent player" + String.valueOf(i+1));
    }
  }
}
