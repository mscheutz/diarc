/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;

public class MockOnRobotScrewingComponent extends DiarcComponent implements OnRobotScrewingComponentInterface{
    @Override
    public boolean configureScrewdriverProgram(Symbol screwType, Symbol screwLength, Symbol torqueTarget) {
        return true;
    }
}
