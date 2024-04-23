/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import ai.thinkingrobots.trade.TRADEService;

public interface PLCComponentInterface {
    @TRADEService
    boolean conveyorForward();

    @TRADEService
    boolean conveyorReverse();
}
