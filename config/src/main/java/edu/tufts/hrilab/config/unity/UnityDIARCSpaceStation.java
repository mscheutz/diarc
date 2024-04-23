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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.Arrays;
import java.util.List;


public class UnityDIARCSpaceStation {
  // for logging
  protected static Logger log = LoggerFactory.getLogger(UnityDIARCSpaceStation.class);

  public static void main(String[] args) {
    List<String> largs = Arrays.asList(args);
    String unityIP = "127.0.0.1";

    if (largs.contains("-unity")) {
      unityIP = largs.get(largs.indexOf("-unity") + 1);
    } else {
      log.warn("No unity IP provided, using default: " + unityIP);
    }

    String goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(robot1:agent) -beliefinitfile unity/pr2UnityTeam_smm.pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    if (largs.contains("--autonomy") || largs.contains("-a")) {
      goalManagerArgs = "-goal listen(self:agent) -goal initializeTrial(robot1:agent) -goal startAutonomy(robot1:agent) -goal listen(self) -beliefinitfile unity/pr2UnityTeam_smm.pl -dbfile domains/unity/space_station_llm_llama_finetuned.asl core.asl dialogue/nlg.asl dialogue/nlu.asl dialogue/handleSemantics.asl";
    }

    DiarcComponent.createInstance(TLDLParserComponent.class, "-dict pr2Unity.dict");
    DiarcComponent.createInstance(edu.tufts.hrilab.map.MapComponent.class, "-groups agent:robot1 -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:robot1 -point_dist_thresh 0.5");
    DiarcComponent.createInstance(edu.tufts.hrilab.tf.TFComponent.class, " -groups agent:robot1");

    DiarcComponent.createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    DiarcComponent.createInstance(ReferenceResolutionComponent.class, "");
    DiarcComponent.createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class, "");
    DiarcComponent.createInstance(SimpleNLGComponent.class, "");
    DiarcComponent.createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class, "");

    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityComponent.class, "-unityip " + unityIP + " -groups agent:robot1");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.space_station.UnitySpaceStation.class, "-agent spacestation -groups agent:robot1");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.space_station.llm.UnitySpaceStationLLM.class, "-groups agent:robot1 -refs refs/unity_space_station_tube_positions.json");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityAgent.class, "-agent rover -groups agent:robot1");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityPR2.class, "-agent robot1 -groups agent:robot1");

    DiarcComponent.createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class, goalManagerArgs);
  }
}
