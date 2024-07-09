/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.diarc.DiarcComponent;

import edu.tufts.hrilab.movebase.MoveBaseComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.Arrays;
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
    String nullListener = "";

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

    if (largs.contains("-nullListener")) {
      nullListener = " -nullListener";
    }

    String goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(" + agentName + ":agent) -beliefinitfile unity/pr2UnityTeam_smm_" + agentName + ".pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    if (largs.contains("--autonomy") || largs.contains("-a")) {
      goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(" + agentName + ":agent) -goal startAutonomy(" + agentName + ":agent) -beliefinitfile unity/pr2UnityTeam_smm.pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    }

    DiarcComponent.createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, "-tldl pr2Unity.dict -llm spaceStationLLMParser -cacheName unity_space_station -cachePersist false -noConfirmation -tldlNoUpdateAddressee");

    DiarcComponent.createInstance(edu.tufts.hrilab.map.MapComponent.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:" + agentName + " -point_dist_thresh 0.5");
    DiarcComponent.createInstance(edu.tufts.hrilab.tf.TFComponent.class, " -groups agent:"  + agentName);

    DiarcComponent.createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    DiarcComponent.createInstance(ReferenceResolutionComponent.class, "");
    DiarcComponent.createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class, "");
    DiarcComponent.createInstance(SimpleNLGComponent.class, "");
    DiarcComponent.createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class, "");

    DiarcComponent.createInstance(edu.tufts.hrilab.llm.LLMComponent.class, "-groups agent:" + agentName + " -service llama -model 7B -endpoint " + llmEndpoint + " -stopwords }");

    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityComponent.class, "-unityip " + unityIP  + " -unityport " + unityPort + " -groups agent:"  + agentName);
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.space_station.UnitySpaceStation.class, "-agent spacestation -groups agent:"  + agentName);
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM.class, "-groups agent:" + agentName + " -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityAgent.class, "-agent rover -groups agent:" + agentName);
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityPR2.class, "-agent " + agentName + " -groups agent:"  + agentName + nullListener );

    DiarcComponent.createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class, goalManagerArgs);
  }
}
