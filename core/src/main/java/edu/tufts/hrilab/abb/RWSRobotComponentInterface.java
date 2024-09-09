/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.cognex.consultant.CognexConsultant;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Effect;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.cognex.consultant.CognexReference;
import edu.tufts.hrilab.cognex.consultant.CognexResult;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.util.List;
import java.util.Map;

public interface RWSRobotComponentInterface {

  @TRADEService
  @Action
  public Symbol addCameraPose(String propertyName, String poseString);

  @TRADEService
  @Action
  public void addDropoffPose(String propertyName, String cameraPoseName, String poseString);

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "cancelCurrentRapidAction()")
  @Effect(effect = {"noRapidError()"}, type = EffectType.SUCCESS, observable = {"noRapidError()"})
  public void goToCameraPose(Symbol locationRef);

  @TRADEService
  @Action
  public void calibrateGripper();

  @TRADEService
  @Action
  public void openGripperRapid();

  @TRADEService
  @Action
  public void closeGripperRapid();

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "cancelCurrentRapidAction()")
  @Effect(effect = {"noRapidError()"}, type = EffectType.SUCCESS, observable = {"noRapidError()"})
  public void pourSauce();

  @TRADEService
  @Action
  public void setGraspPoint(Symbol itemType, String targetString);

  @TRADEService
  @Action
  public boolean defineGraspPointForDescriptor(Symbol refId, Symbol itemType);

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "cancelCurrentRapidAction()")
  @Effect(effect = {"noRapidError()"}, type = EffectType.SUCCESS, observable = {"noRapidError()"})
  public boolean perceiveEntityFromSymbol(Symbol refId);

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "cancelCurrentRapidAction()")
  @Effect(effect = {"noRapidError()"}, type = EffectType.SUCCESS, observable = {"noRapidError()"})
  public void pickupItem(Symbol refId);

  @TRADEService
  @Action
  public Justification cookItem(Symbol refId, Term time);

  @TRADEService
  @Action
  public Justification sauteItem(Symbol refId, Term time);

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "cancelCurrentRapidAction()")
  @Effect(effect = {"noRapidError()"}, type = EffectType.SUCCESS, observable = {"noRapidError()"})
  public boolean goToPoseName(String poseName);

  @TRADEService
  @Action
  @OnInterrupt(onCancelServiceCall = "cancelCurrentRapidAction()")
  @Effect(effect = {"noRapidError()"}, type = EffectType.SUCCESS, observable = {"noRapidError()"})
  public void putDownItem(Symbol refId, Symbol pose);

  public void moveToObject(Symbol refId);

  public void setCognexConsultant(CognexConsultant cognexConsultant);

  @TRADEService
  @Action
  public boolean cancelCurrentRapidAction();

  @TRADEService
  @Observes({"noRapidError()"})
  List<Map<Variable, Symbol>> checkError(Term t);

  @TRADEService
  @Action
  public CognexResult getMatchingResult(CognexReference toReBind, List<CognexResult> results);
}
