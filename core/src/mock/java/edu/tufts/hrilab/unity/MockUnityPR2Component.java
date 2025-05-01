/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Condition;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.unity.space_station.SpaceStationObservers;
import edu.tufts.hrilab.unity.space_station.TubeNameHelpers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MockUnityPR2Component extends DiarcComponent implements UnityPR2Interface {

  private SpaceStationObservers observers;

  @Override
  public void init() {
    observers = new SpaceStationObservers();
    observers.registerWithTRADE(this.getMyGroups());
  }

  @Override
  public Symbol getPR2Area() {
    return null;
  }

  @Override
  public boolean startPR2Repair(Symbol target) {
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("repairMockTube").argTypes(Symbol.class))
              .call(Boolean.class, target);
    } catch (TRADEException e) {
      log.error("Error calling repairMockTube.", e);
      return false;
    }

    return true;
  }

  @Override
  public boolean stopPR2Repair(Symbol target) {
    return true;
  }

}
