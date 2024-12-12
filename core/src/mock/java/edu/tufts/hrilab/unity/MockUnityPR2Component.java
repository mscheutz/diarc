/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;

public class MockUnityPR2Component extends DiarcComponent implements UnityPR2Interface {

    @Override
    public Symbol getPR2Area() {
        return null;
    }

    @Override
    public boolean startPR2Repair(Symbol target) {
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("repairMockTube").argTypes(Symbol.class))
                .call(Boolean.class, target);
      } catch (TRADEException e) {
        log.error("Error calling repairMockTube.", e);
        return false;
      }

      return true;
    }

    @Override
    public boolean stopPR2Repair(Symbol target) {
        return true;
    }
}
