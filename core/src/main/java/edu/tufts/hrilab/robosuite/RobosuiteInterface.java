package edu.tufts.hrilab.robosuite;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

public interface RobosuiteInterface {

  @Action
  @TRADEService
  public String diarc_step(String action);

}
