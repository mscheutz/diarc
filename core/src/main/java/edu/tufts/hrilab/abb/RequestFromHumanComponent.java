/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.abb;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.List;

public class RequestFromHumanComponent extends DiarcComponent {

  Symbol humanSpeaker;
  Symbol robotSpeaker;

  @Override
  protected void init() {
    //Do we want to get this agent's group name? Different hardcoded name? Statically indexed?
    //Actually, the speaker here is the stationary yumi and not actually the human right
    humanSpeaker = Factory.createSymbol("human");
    robotSpeaker = Factory.createSymbol("self");
  }

  @TRADEService
  @Action
  public String getQuestionFromRefs(Symbol itemType, Symbol fromPoseRef, Symbol toPoseRef) {
    String itemString = itemType.getName();
    String fromPoseString = fromPoseRef.getName();
    String toPoseString = toPoseRef.getName();
    try {
      List<Term> refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class)).call(List.class, fromPoseRef);
      if (refs != null && refs.size() > 0) {
        fromPoseString = refs.get(0).getName();
      }
    } catch (TRADEException e) {
      log.error("Error calling getProperties for fromPoseRef", e);
    }
    try {
      List<Term> refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("getProperties").argTypes(Symbol.class)).call(List.class, toPoseRef);
      toPoseString = refs.get(0).getName();
    } catch (TRADEException e) {
      log.error("Error calling getProperties for toPoseRef", e);
    }
    return "Could you please bring a " + itemString + " from the " + fromPoseString + " to the " + toPoseString;
  }
}