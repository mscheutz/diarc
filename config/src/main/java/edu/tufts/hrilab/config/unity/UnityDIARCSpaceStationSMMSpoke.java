/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.diarc.DiarcComponent;

import edu.tufts.hrilab.movebase.MoveBaseComponent;
import edu.tufts.hrilab.tf.TFComponent;
import edu.tufts.hrilab.unity.UnityPR2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.Arrays;
import java.util.List;


public class UnityDIARCSpaceStationSMMSpoke {
    // for logging
    protected static Logger log = LoggerFactory.getLogger(UnityDIARCSpaceStationSMMSpoke.class);

    public static void main(String[] args) {
        List<String> largs = Arrays.asList(args);
        String agentName = "robot2";

        if (largs.contains("-agent")) {
            agentName = largs.get(largs.indexOf("-agent") + 1);
        }

        DiarcComponent.createInstance(MoveBaseComponent.class, "-groups agent:" + agentName + " -point_dist_thresh 0.5");
        DiarcComponent.createInstance(TFComponent.class, " -groups agent:" + agentName);

        DiarcComponent.createInstance(UnityPR2.class, "-agent " + agentName + " -groups agent:" + agentName);
    }
}