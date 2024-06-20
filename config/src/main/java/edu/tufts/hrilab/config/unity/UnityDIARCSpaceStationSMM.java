/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.diarc.DiarcComponent;

import edu.tufts.hrilab.movebase.MoveBaseComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.tf.TFComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.unity.UnityComponent;
import edu.tufts.hrilab.unity.space_station.UnitySpaceStation;
import edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM;
import edu.tufts.hrilab.unity.UnityAgent;
import edu.tufts.hrilab.unity.UnityPR2;
import edu.tufts.hrilab.action.GoalManagerImpl;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.Arrays;
import java.util.List;


public class UnityDIARCSpaceStationSMM {
  // for logging
  protected static Logger log = LoggerFactory.getLogger(UnityDIARCSpaceStationSMM.class);

  public static void main(String[] args) {
    List<String> largs = Arrays.asList(args);
    String unityIP = "127.0.0.1";

    if (largs.contains("-unity")) {
      unityIP = largs.get(largs.indexOf("-unity") + 1);
    } else {
      log.warn("No unity IP provided, using default: " + unityIP);
    }

    String goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(robot1:agent) -goal initializeTrial(robot2:agent) -beliefinitfile unity/pr2UnityTeam_smm.pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    if (largs.contains("--autonomy") || largs.contains("-a")) {
      goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(robot1:agent) -goal initializeTrial(robot2:agent) -goal startAutonomy(robot1:agent) -goal listen(self) -beliefinitfile unity/pr2UnityTeam_smm.pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    }

    DiarcComponent.createInstance(TLDLParserComponent.class, "-dict pr2Unity.dict");

    DiarcComponent.createInstance(MapComponent.class, "-groups agent:robot1 -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:robot1 -point_dist_thresh 0.5");
    DiarcComponent.createInstance(TFComponent.class, " -groups agent:robot1");

    DiarcComponent.createInstance(MapComponent.class, "-groups agent:robot2 -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:robot2 -point_dist_thresh 0.5");
    DiarcComponent.createInstance(TFComponent.class, " -groups agent:robot2");

    DiarcComponent.createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    DiarcComponent.createInstance(ReferenceResolutionComponent.class, "");
    DiarcComponent.createInstance(DialogueComponent.class, "");
    DiarcComponent.createInstance(SimpleNLGComponent.class, "");
    DiarcComponent.createInstance(ListenerComponent.class, "");

    DiarcComponent.createInstance(UnityComponent.class, "-unityip " + unityIP + " -groups agent:robot1 agent:robot2");
    DiarcComponent.createInstance(UnitySpaceStation.class, "-agent spacestation -groups agent:robot1 agent:robot2");
    DiarcComponent.createInstance(UnitySpaceStationLLM.class, "-groups agent:robot1 agent:robot2 -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(UnityAgent.class, "-agent rover -groups agent:robot1 agent:robot2");
    DiarcComponent.createInstance(UnityPR2.class, "-agent robot1 -groups agent:robot1");
    DiarcComponent.createInstance(UnityPR2.class, "-agent robot1 -groups agent:robot2");

    DiarcComponent.createInstance(GoalManagerImpl.class, goalManagerArgs);
  }
}