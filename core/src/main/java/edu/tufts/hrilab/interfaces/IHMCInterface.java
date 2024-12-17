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
  void test1(Symbol test_symbol);


  @TRADEService
  @Action
  @Effect(
          effect={"at(?actor,?location)"},
          type = EffectType.SUCCESS,
          observable = {"at(?actor,?location"}
  )
  @OnInterrupt(onCancelServiceCall = "stop()", onSuspendServiceCall = "stop()")
  Justification test2(Symbol location);


  @TRADEService
  @Action
  public void open_door();
}
