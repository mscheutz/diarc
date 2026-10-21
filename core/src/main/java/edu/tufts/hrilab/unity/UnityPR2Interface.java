package edu.tufts.hrilab.unity;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;

public interface UnityPR2Interface {

  @TRADEService
  @Action
  Symbol getPR2Area();

  @TRADEService
  @Action
  boolean startPR2Repair(Symbol target);

  @TRADEService
  @Action
  boolean stopPR2Repair(Symbol target);

}
