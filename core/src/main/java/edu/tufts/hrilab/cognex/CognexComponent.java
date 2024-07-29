/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.cognex;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.cognex.consultant.CognexConsultant;
import edu.tufts.hrilab.diarc.DiarcComponent;

import java.util.List;

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
