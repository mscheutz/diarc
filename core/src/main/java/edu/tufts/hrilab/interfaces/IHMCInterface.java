package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

public interface IHMCInterface {
  @TRADEService
  @Action
  public void walk();

  @TRADEService
  @Action
  public void stop();

  @TRADEService
  @Action
  public void go_to(Float x, Float y);

  @TRADEService
  @Action
  public void go_to(String location);
}
