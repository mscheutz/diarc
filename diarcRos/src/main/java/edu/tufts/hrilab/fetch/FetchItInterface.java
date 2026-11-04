/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Condition;
import edu.tufts.hrilab.action.annotations.Effect;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

public interface FetchItInterface extends FetchInterface {

  @TRADEService
  @Action
  Justification moveObjectAbove(Symbol objectRef_0, Symbol objectRef_1, String groupName);


  @TRADEService
  @Action
  @Condition(
          condition={"grasping(?actor,?objectRef,?arm)"},
          type= ConditionType.PRE,
          observable={"grasping(?actor,?objectRef,?arm)"}
  )
  @Effect(
          effect={"grasping(?actor,?objectRef,?arm)"},
          type = EffectType.SUCCESS,
          observable={"grasping(?actor,?objectRef,?arm)"}
  )
  Justification moveObjectFetchItPrimitive(Symbol objectRef, String arm, String direction);
}
