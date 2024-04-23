/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

public interface KolverComponentInterface {
    int incrementId();

    @TRADEService
    boolean configureScrewdriverParam(Term screwType, Symbol val);

    @TRADEService
    int getScrewdriverProgramIdFromSymbol(Symbol s);

    @TRADEService
    boolean configureScrewdriverProgram(Symbol screwType, Symbol torqueTarget, Symbol torqueMax,
                                        Symbol angleMax);

    @TRADEService
    boolean runScrewdriverProgram(int programID);

    @TRADEService
    boolean switchProgram(int id);

    @TRADEService
    boolean resetScrewdriver();
}
