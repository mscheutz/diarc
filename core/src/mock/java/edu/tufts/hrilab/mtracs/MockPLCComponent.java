/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import edu.tufts.hrilab.diarc.DiarcComponent;

/**
 * MockPLCComponent represents a PLC unit, specifically equipped with a program that allows it to drive
 * a conveyor belt.
 */
public class MockPLCComponent extends DiarcComponent implements PLCComponentInterface {

    /**
     * Moves the conveyor belt that <code>this</code> PLCComponent is connected to forward.
     *
     * @return True if this move was executed successfully, false if otherwise.
     */
    @Override
    public boolean conveyorForward() {
        return true;
    }

    /**
     * Moves the conveyor belt that <code>this</code> PLCComponent is connected to backwards.
     *
     * @return True if this move was executed successfully, false if otherwise.
     */
    @Override
    public boolean conveyorReverse() {
        return true;
    }

}
