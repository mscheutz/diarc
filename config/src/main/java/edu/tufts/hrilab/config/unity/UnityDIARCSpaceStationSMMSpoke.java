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


public class UnityDIARCSpaceStationSMMSpoke {
    // for logging
    protected static Logger log = LoggerFactory.getLogger(UnityDIARCSpaceStationSMMSpoke.class);

    public static void main(String[] args) {
        List<String> largs = Arrays.asList(args);

        DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:robot2 -point_dist_thresh 0.5");
        DiarcComponent.createInstance(TFComponent.class, " -groups agent:robot2");

        DiarcComponent.createInstance(UnityPR2.class, "-agent robot2 -groups agent:robot2");
    }
}