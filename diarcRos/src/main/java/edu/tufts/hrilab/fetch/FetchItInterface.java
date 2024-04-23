/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

public interface FetchItInterface extends FetchInterface {

  @TRADEService
  @Action
  Justification moveObjectAbove(Symbol objectRef_0, Symbol objectRef_1, String groupName);
}
