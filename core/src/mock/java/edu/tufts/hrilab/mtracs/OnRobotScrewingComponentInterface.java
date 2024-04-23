/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Symbol;

public interface OnRobotScrewingComponentInterface {

    @TRADEService
    boolean configureScrewdriverProgram(Symbol screwType, Symbol screwLength, Symbol torqueTarget);
}
