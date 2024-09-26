/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.unity;

import edu.tufts.hrilab.diarc.DiarcComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.List;

public class UnityDIARCSpaceStationExample {
  // for logging
  protected static Logger log = LoggerFactory.getLogger(UnityDIARCSpaceStationExample.class);

  // start the configuration
  public static void main(String[] args) {
    List<String> largs = Arrays.asList(args);
    String unityIP = "127.0.0.1";
    String unityPort = "1755";

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

    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityComponent.class, "-unityip " + unityIP  + " -unityport " + unityPort);
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.space_station.UnitySpaceStation.class, "-agent spacestation");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityAgent.class, "-agent rover");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityPR2.class, "-agent robot1");
    DiarcComponent.createInstance(edu.tufts.hrilab.unity.UnityPR2.class, "-agent robot2");
  }
}