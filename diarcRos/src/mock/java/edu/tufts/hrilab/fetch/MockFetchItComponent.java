/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

public class MockFetchItComponent extends MockFetchComponent implements FetchItInterface {

  public Justification moveObjectAbove(Symbol objectRef_0, Symbol objectRef_1, String groupName) {
    log.info("[moveObjectAbove]");
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveObjectFetchItPrimitive(Symbol objectRef, String groupName, String direction) {
    log.info("[moveObject]");
    simExecTime();
    return new ConditionJustification(true);
  }
}
