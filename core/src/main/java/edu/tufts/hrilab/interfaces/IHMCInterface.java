package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Effect;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

public interface IHMCInterface extends MoveBaseInterface {
  @TRADEService
  @Action
  public void walk();


  @TRADEService
  @Action
  public Justification push_door_primitive();


  @TRADEService
  @Action
  public Justification pull_door_primitive();
}
