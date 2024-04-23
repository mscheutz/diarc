/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.arm;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.interfaces.ArmInterface;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;

public interface MockArmInterface extends ArmInterface {
    @TRADEService
    @Action
    boolean startPouring(double ang);

    @TRADEService
    @Action
    boolean startPouring(Symbol obj);

    @TRADEService
    @Action
    boolean stopPouring(Symbol obj);

    @TRADEService
    @Action
    boolean stopPouring();

    @TRADEService
    @Action
    boolean moveAbove(Symbol s, Symbol s2);

}
