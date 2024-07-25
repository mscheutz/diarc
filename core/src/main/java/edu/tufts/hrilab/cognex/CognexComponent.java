/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.cognex;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.cognex.consultant.CognexConsultant;
import edu.tufts.hrilab.cognex.consultant.CognexJob;
import edu.tufts.hrilab.cognex.consultant.CognexReference;
import edu.tufts.hrilab.cognex.consultant.CognexResult;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class CognexComponent extends DiarcComponent {

  private CognexConsultant consultant;

  public void init() {
    consultant = new CognexConsultant();
    List<String> groups = this.getMyGroups();
    groups.add("physobj");//todo: should this should be adjusted based on robot prefix if there is more than one?
    try {
      TRADE.registerAllServices(consultant, groups);
    } catch (TRADEException e) {
      log.error("[init] error registering CognexConsultant with TRADE", e);
    }
  }

}
